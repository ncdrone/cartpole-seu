#!/usr/bin/env bash
# Rewrite the Dockerfile FROM line with the base image's current content digest, so every build
# of this repo uses the identical glibc/libm/GCC. Needs a running Docker daemon.
#   bash bench/pin_image.sh
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DF="$ROOT/Dockerfile"
BASE="debian:bookworm-slim"
docker pull "$BASE" >/dev/null
DIGEST="$(docker inspect --format='{{index .RepoDigests 0}}' "$BASE")"
[ -n "$DIGEST" ] || { echo "could not read digest"; exit 1; }
# bash 3.2-safe in-place edit via python
python3 - "$DF" "$DIGEST" <<'EOF'
import re, sys
p, digest = sys.argv[1], sys.argv[2]
s = open(p).read()
s2 = re.sub(r"^FROM debian:bookworm-slim(@sha256:[0-9a-f]+)?$", f"FROM {digest}", s, count=1, flags=re.M)
open(p, "w").write(s2)
print("pinned:", digest)
EOF
