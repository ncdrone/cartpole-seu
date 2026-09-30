# Pinned software constraint for reproducible runs.
# Same image on Docker-for-Mac (arm64), Docker on Jetson/Pi (arm64), or amd64 CI:
#   same GCC, same glibc/libm, same flags  =>  bit-identical CSV on the same architecture.
# Cross-architecture (arm64 vs amd64) differs only at libm ULP level; the fault list and outcomes match.
#
#   docker build -t cartpole .
#   docker run --rm cartpole                                   # build_demo verify + repro hash
#   docker run --rm -p 8080:8080 cartpole serve                # LAN viewer from the container
#   docker run --rm cartpole backtest sim/scenarios/grid.json  # any harness command
#   docker build --platform linux/amd64 -t cartpole:amd64 .    # x86 variant on an Apple Silicon Mac
#
# Pin by digest before shipping: `bash scripts/pin_image.sh` rewrites the FROM line with the current digest.
FROM debian@sha256:3783cc01769c7b2b1b83a5c5ad96c815348e28ed7da68e2e3687004faa906251

RUN apt-get update && apt-get install -y --no-install-recommends \
        g++ cmake make python3 ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work
COPY fsw/ fsw/
COPY sim/ sim/
COPY scripts/ scripts/
COPY bench/ bench/
COPY docs/GLOSSARY.md docs/GLOSSARY.md

# Build at image build time so a broken toolchain fails here. The committed goldens are set aside during this build:
# comparing against them is the job of `docker run cartpole tests`, and a golden that predates a controller change
# must not make the image itself unbuildable.
RUN mv sim/golden /tmp/golden && mkdir sim/golden \
    && bash scripts/build_demo.sh --clean --no-install \
    && rm -rf sim/golden && mv /tmp/golden sim/golden \
    && python3 - <<'EOF'
import json; p="fsw/build/build_info.json"; d=json.load(open(p)); d["container"]=True; json.dump(d, open(p,"w"), indent=2)
EOF

EXPOSE 8080
COPY scripts/docker-entry.sh /usr/local/bin/entry
ENTRYPOINT ["entry"]
CMD ["verify"]
