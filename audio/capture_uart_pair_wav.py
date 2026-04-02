# =========================
# Example commands (PowerShell)
# =========================
# python capture_uart_pair_wav.py --port COM6 --seconds 20 --base [what I want file name to be]
# =========================

import argparse
import serial
import wave
import struct
import time

SYNC = 0xA5

def write_wav(path, samples, fs=48000):
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(fs)
        wf.writeframes(struct.pack("<" + "h"*len(samples), *samples))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=3072000)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--fs", type=int, default=48000)
    ap.add_argument("--base", default="capture")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    print(f"Opened {args.port} @ {args.baud} baud")

    deadline = time.time() + args.seconds

    in_samps = []
    out_samps = []

    # Parse: A5 in_lo in_hi out_lo out_hi
    state = 0
    in_lo = in_hi = out_lo = out_hi = 0

    while time.time() < deadline:
        chunk = ser.read(4096)
        if not chunk:
            continue
        for b in chunk:
            if state == 0:
                if b == SYNC:
                    state = 1
            elif state == 1:
                in_lo = b; state = 2
            elif state == 2:
                in_hi = b; state = 3
            elif state == 3:
                out_lo = b; state = 4
            else:  # state == 4
                out_hi = b
                in_sample  = struct.unpack("<h", bytes([in_lo, in_hi]))[0]
                out_sample = struct.unpack("<h", bytes([out_lo, out_hi]))[0]
                in_samps.append(in_sample)
                out_samps.append(out_sample)
                state = 0

    ser.close()
    print(f"Captured {len(in_samps)} paired samples")

    in_path  = args.base + "_in.wav"
    out_path = args.base + "_out.wav"
    write_wav(in_path,  in_samps,  fs=args.fs)
    write_wav(out_path, out_samps, fs=args.fs)
    print("Wrote", in_path)
    print("Wrote", out_path)

if __name__ == "__main__":
    main()
