import argparse
import os
import glob
import numpy as np
import torch
import torchaudio
import torch.nn.functional as F
import inspect
import huggingface_hub
import whisper
import soundfile as sf
from datetime import datetime

from jiwer import (
    Compose,
    ToLowerCase,
    RemovePunctuation,
    RemoveMultipleSpaces,
    Strip,
    ReduceToListOfListOfWords,
    process_words,
)

# ---- SpeechBrain <-> torchaudio compatibility patch ----
if not hasattr(torchaudio, "list_audio_backends"):
    torchaudio.list_audio_backends = lambda: ["soundfile"]

# ---- SpeechBrain <-> huggingface_hub compatibility patch ----
sig = inspect.signature(huggingface_hub.hf_hub_download)
if "use_auth_token" not in sig.parameters:
    _orig_hf_hub_download = huggingface_hub.hf_hub_download

    def hf_hub_download_compat(*args, use_auth_token=None, token=None, **kwargs):
        if token is None and use_auth_token is not None:
            token = use_auth_token
        return _orig_hf_hub_download(*args, token=token, **kwargs)

    huggingface_hub.hf_hub_download = hf_hub_download_compat

from speechbrain.inference.speaker import EncoderClassifier

# ---- SpeechBrain optional custom.py fetch patch ----
try:
    import speechbrain.inference.interfaces as sb_inf_if
    from huggingface_hub.errors import RemoteEntryNotFoundError
except Exception:
    sb_inf_if = None
    RemoteEntryNotFoundError = None

if sb_inf_if is not None and hasattr(sb_inf_if, "fetch") and RemoteEntryNotFoundError is not None:
    _orig_sb_fetch = sb_inf_if.fetch

    def fetch_compat(filename, *args, **kwargs):
        try:
            return _orig_sb_fetch(filename, *args, **kwargs)
        except RemoteEntryNotFoundError as e:
            if str(filename) == "custom.py":
                raise ValueError(str(e))
            raise

    sb_inf_if.fetch = fetch_compat


# -----------------------------
# Logging utilities
# -----------------------------
def make_logger(log_path: str):
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    f = open(log_path, "w", encoding="utf-8")

    def log(msg: str = ""):
        print(msg, flush=True)
        f.write(msg + "\n")
        f.flush()

    return log, f


def derive_method_name(input_wav: str, output_wav: str) -> str:
    """
    Derive a single method name from input/output filenames by removing trailing _in/_out (or -in/-out).
    Example:
      pitch_test_sw7_in.wav + pitch_test_sw7_out.wav -> pitch_test_sw7
    """

    def strip_suffix(name: str) -> str:
        for suf in ["_in", "_out", "-in", "-out"]:
            if name.endswith(suf):
                return name[: -len(suf)]
        return name

    in_base = os.path.splitext(os.path.basename(input_wav))[0]
    out_base = os.path.splitext(os.path.basename(output_wav))[0]

    in_core = strip_suffix(in_base)
    out_core = strip_suffix(out_base)

    if in_core == out_core and in_core:
        return in_core.rstrip("_-")

    # Fallback: longest common prefix, then trim trailing separators
    common = os.path.commonprefix([in_core, out_core]).rstrip("_-")
    return common if common else in_core.rstrip("_-")


# -----------------------------
# Audio utilities
# -----------------------------
def load_audio_mono_resample(path: str, target_sr: int = 16000) -> tuple[torch.Tensor, int]:
    """
    IMPORTANT: Uses soundfile to avoid torchaudio.load() -> TorchCodec dependency.
    Returns mono float tensor [T] and sample rate.
    """
    audio, sr = sf.read(path, dtype="float32", always_2d=True)  # [T, C]
    audio = audio.mean(axis=1)  # mono [T]

    wav = torch.from_numpy(audio).unsqueeze(0)  # [1, T]
    if sr != target_sr:
        wav = torchaudio.functional.resample(wav, sr, target_sr)
        sr = target_sr

    return wav.squeeze(0), sr  # [T], sr


def chunk_waveform(wav: torch.Tensor, sr: int, chunk_sec: float = 3.0, hop_sec: float = 0.5) -> list[torch.Tensor]:
    chunk_len = int(chunk_sec * sr)
    hop_len = int(hop_sec * sr)
    if wav.numel() < chunk_len:
        return [wav]

    chunks = []
    for start in range(0, wav.numel() - chunk_len + 1, hop_len):
        chunks.append(wav[start : start + chunk_len])
    if not chunks:
        chunks = [wav]
    return chunks


# -----------------------------
# ASR + WER
# -----------------------------
def transcribe_whisper(model, wav_path: str, language: str | None = None) -> str:
    kwargs = {"fp16": torch.cuda.is_available()}
    if language:
        kwargs["language"] = language
        kwargs["task"] = "transcribe"
    out = model.transcribe(wav_path, **kwargs)
    return (out.get("text") or "").strip()


def compute_wer_breakdown(ref_text: str, hyp_text: str) -> dict:
    """
    Returns dict with S, D, I, N (reference words), and WER.
    """
    transform = Compose(
        [ToLowerCase(), RemovePunctuation(), RemoveMultipleSpaces(), Strip(), ReduceToListOfListOfWords()]
    )
    out = process_words(
        ref_text,
        hyp_text,
        reference_transform=transform,
        hypothesis_transform=transform,
    )

    S = int(out.substitutions)
    D = int(out.deletions)
    I = int(out.insertions)
    H = int(out.hits)

    # N is number of words in reference = H + S + D
    N = int(H + S + D)

    wer = (S + D + I) / N if N > 0 else 0.0

    return {
        "S": S,
        "D": D,
        "I": I,
        "H": H,
        "N": N,
        "WER": float(wer),
    }


# -----------------------------
# Speaker embeddings + EER
# -----------------------------
@torch.inference_mode()
def embed_chunks(classifier: EncoderClassifier, chunks: list[torch.Tensor], device: str = "cpu") -> torch.Tensor:
    batch = torch.stack(chunks).to(device)
    if batch.dtype != torch.float32:
        batch = batch.float()

    batch = batch / (batch.abs().amax(dim=1, keepdim=True).clamp(min=1e-6))

    emb = classifier.encode_batch(batch)  # [B, 1, D] often
    emb = emb.squeeze(1) if emb.ndim == 3 else emb
    emb = F.normalize(emb, p=2, dim=1)
    return emb


def cosine_scores(enroll_emb: torch.Tensor, test_embs: torch.Tensor) -> np.ndarray:
    if enroll_emb.ndim == 1:
        enroll_emb = enroll_emb.unsqueeze(0)
    enroll_emb = F.normalize(enroll_emb, p=2, dim=1)
    test_embs = F.normalize(test_embs, p=2, dim=1)
    scores = F.cosine_similarity(test_embs, enroll_emb.repeat(test_embs.size(0), 1), dim=1)
    return scores.detach().cpu().numpy()


def score_stats(arr: np.ndarray) -> dict:
    arr = np.asarray(arr)
    return {
        "n": int(arr.size),
        "mean": float(arr.mean()) if arr.size else float("nan"),
        "std": float(arr.std()) if arr.size else float("nan"),
        "min": float(arr.min()) if arr.size else float("nan"),
        "max": float(arr.max()) if arr.size else float("nan"),
    }


def compute_eer_with_details(genuine_scores: np.ndarray, impostor_scores: np.ndarray) -> dict:
    genuine_scores = np.asarray(genuine_scores)
    impostor_scores = np.asarray(impostor_scores)

    thresholds = np.sort(np.unique(np.concatenate([genuine_scores, impostor_scores])))

    best = None
    N = genuine_scores.size
    M = impostor_scores.size

    for t in thresholds:
        fa = int((impostor_scores >= t).sum())   # false accepts
        fr = int((genuine_scores < t).sum())     # false rejects

        FAR = fa / M if M > 0 else 0.0
        FRR = fr / N if N > 0 else 0.0
        diff = abs(FAR - FRR)
        eer = 0.5 * (FAR + FRR)

        cand = {
            "threshold": float(t),
            "EER": float(eer),
            "FAR": float(FAR),
            "FRR": float(FRR),
            "false_accepts": fa,
            "false_rejects": fr,
            "num_impostors": int(M),
            "num_genuine": int(N),
            "abs_far_minus_frr": float(diff),
        }

        if best is None or cand["abs_far_minus_frr"] < best["abs_far_minus_frr"]:
            best = cand

    return best


def gather_impostor_embeddings(
    classifier,
    impostors_dir: str,
    max_files: int = 146,
    chunk_sec: float = 3.0,
    hop_sec: float = 0.5,
    device: str = "cpu",
    log=lambda *_: None,
) -> torch.Tensor:
    wav_paths = sorted(glob.glob(os.path.join(impostors_dir, "*.wav")))
    if len(wav_paths) == 0:
        raise ValueError(f"No .wav files found in impostors_dir={impostors_dir}")

    wav_paths = wav_paths[:max_files]
    all_embs = []

    log(f"Embedding {len(wav_paths)} impostor files...")
    for i, p in enumerate(wav_paths, 1):
        wav, sr = load_audio_mono_resample(p, 16000)
        chunks = chunk_waveform(wav, sr, chunk_sec, hop_sec)
        embs = embed_chunks(classifier, chunks, device=device)
        all_embs.append(embs.mean(dim=0, keepdim=True))

        if i % 10 == 0 or i == len(wav_paths):
            log(f"  impostors: {i}/{len(wav_paths)}")

    return torch.cat(all_embs, dim=0)


# -----------------------------
# Main
# -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input_wav", required=True)
    ap.add_argument("--output_wav", required=True)
    ap.add_argument("--ref_text", default=None, help="If provided, use as WER reference. Otherwise ASR(input) is used.")
    ap.add_argument("--whisper_model", default="base")
    ap.add_argument("--language", default=None, help="Optional language code (e.g. en).")
    ap.add_argument("--impostors_dir", required=True)
    ap.add_argument("--max_impostors", type=int, default=146)
    args = ap.parse_args()

    chunk_sec = 3.0
    hop_sec = 0.5

    device = "cuda" if torch.cuda.is_available() else "cpu"

    method = derive_method_name(args.input_wav, args.output_wav)
    log_path = os.path.join("results", f"{method}_metrics.txt")
    log, f = make_logger(log_path)

    try:
        log(f"Log file: {log_path}")
        log(f"Device: cpu" if device == "cpu" else f"Device: {device}")
        log(f"Input WAV:  {args.input_wav}")
        log(f"Output WAV: {args.output_wav}")
        log(f"Impostors dir: {args.impostors_dir} | max_impostors={args.max_impostors}")
        log(f"Whisper model: {args.whisper_model}")
        log(f"Chunking: chunk_sec={chunk_sec}, hop_sec={hop_sec}")
        log("")

        # ---- WER ----
        log(f"Loading Whisper model '{args.whisper_model}'...")
        asr = whisper.load_model(args.whisper_model)
        log("Whisper loaded.")

        log("Transcribing input...")
        in_text = transcribe_whisper(asr, args.input_wav, language=args.language)
        log("Transcribing output...")
        out_text = transcribe_whisper(asr, args.output_wav, language=args.language)

        log("\n=== ASR TRANSCRIPTS ===")
        log(f"[input ASR]  {in_text}")
        log(f"[output ASR] {out_text}")

        ref = args.ref_text.strip() if args.ref_text else in_text
        hyp = out_text

        log("\n=== WER BREAKDOWN ===")
        log("WER = (S + D + I) / N, where N = #words in reference")
        wer_info = compute_wer_breakdown(ref, hyp)

        S, D, I, N = wer_info["S"], wer_info["D"], wer_info["I"], wer_info["N"]
        wer_val = wer_info["WER"]

        log(f"S (substitutions): {S}")
        log(f"D (deletions):     {D}")
        log(f"I (insertions):    {I}")
        log(f"N (ref words):     {N}")
        log(f"WER = ({S} + {D} + {I}) / {N} = {wer_val:.4f}")

        # ---- Speaker Verification / EER ----
        log("\nLoading speaker embedding model (SpeechBrain ECAPA)...")
        classifier = EncoderClassifier.from_hparams(
            source="speechbrain/spkrec-ecapa-voxceleb",
            run_opts={"device": device},
        )
        log("Speaker model loaded.")

        log("Loading + chunking input/output audio for embeddings...")
        in_wav, sr = load_audio_mono_resample(args.input_wav, 16000)
        out_wav, _ = load_audio_mono_resample(args.output_wav, 16000)

        in_chunks = chunk_waveform(in_wav, sr, chunk_sec, hop_sec)
        out_chunks = chunk_waveform(out_wav, sr, chunk_sec, hop_sec)
        log(f"Input chunks: {len(in_chunks)} | Output chunks: {len(out_chunks)}")

        log("Embedding input chunks (enrollment + baseline)...")
        in_embs = embed_chunks(classifier, in_chunks, device=device)
        enroll_emb = in_embs.mean(dim=0)

        baseline_genuine = cosine_scores(enroll_emb, in_embs)

        log("Embedding output chunks (anon scores)...")
        out_embs = embed_chunks(classifier, out_chunks, device=device)
        anon_scores = cosine_scores(enroll_emb, out_embs)

        impostor_embs = gather_impostor_embeddings(
            classifier,
            args.impostors_dir,
            max_files=args.max_impostors,
            chunk_sec=chunk_sec,
            hop_sec=hop_sec,
            device=device,
            log=log,
        )
        impostor_scores = cosine_scores(enroll_emb, impostor_embs)

        log("\n=== SCORE STATISTICS (cosine similarity) ===")
        b = score_stats(baseline_genuine)
        a = score_stats(anon_scores)
        imp = score_stats(impostor_scores)

        log(f"Baseline genuine: n={b['n']}, mean={b['mean']:.4f}, std={b['std']:.4f}, min={b['min']:.4f}, max={b['max']:.4f}")
        log(f"Anon genuine:     n={a['n']}, mean={a['mean']:.4f}, std={a['std']:.4f}, min={a['min']:.4f}, max={a['max']:.4f}")
        log(f"Impostor:         n={imp['n']}, mean={imp['mean']:.4f}, std={imp['std']:.4f}, min={imp['min']:.4f}, max={imp['max']:.4f}")

        log("\nComputing EER (threshold sweep)...")
        base_eer = compute_eer_with_details(baseline_genuine, impostor_scores)
        anon_eer = compute_eer_with_details(anon_scores, impostor_scores)

        def log_eer_block(name: str, d: dict):
            log(f"\n=== {name} EER DETAILS ===")
            log(f"Chosen threshold t*: {d['threshold']:.4f}")
            log(f"Genuine trials N:    {d['num_genuine']}")
            log(f"Impostor trials M:   {d['num_impostors']}")
            log(f"False rejects:       {d['false_rejects']}  -> FRR = {d['FRR']*100:.2f}%")
            log(f"False accepts:       {d['false_accepts']}  -> FAR = {d['FAR']*100:.2f}%")
            log(f"|FAR - FRR|:         {d['abs_far_minus_frr']*100:.2f}%")
            log(f"EER = (FAR + FRR)/2  = ({d['FAR']*100:.2f}% + {d['FRR']*100:.2f}%)/2 = {d['EER']*100:.2f}%")

        log_eer_block("BASELINE (input vs input)", base_eer)
        log_eer_block("ANON (input vs output)", anon_eer)

        log("\nDone.")
        log(f"Saved results to: {log_path}")

    finally:
        f.close()


if __name__ == "__main__":
    main()