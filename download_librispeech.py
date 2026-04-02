import os
from torchaudio.datasets import LIBRISPEECH

root = "data"
os.makedirs(root, exist_ok=True)

splits = ["dev-clean", "dev-other", "test-clean", "test-other"]
for split in splits:
    print("Downloading:", split)
    LIBRISPEECH(root=root, url=split, download=True)

print("Done.")