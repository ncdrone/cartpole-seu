#!/usr/bin/env bash
# Record the golden 1 s demo trace for THIS environment class, plus the repro-scenario result hash.
# Run once per software constraint you want to pin (Mac metal, Pi metal, Jetson metal, the Docker image).
#   bash scripts/make_golden.sh
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/fsw/build"
[ -x "$BUILD/cartpole_demo" ] || { echo "build first: bash scripts/build_demo.sh"; exit 1; }
ENVCLASS="$(sed -n 's/.*"envclass": "\([^"]*\)".*/\1/p' "$BUILD/build_info.json" 2>/dev/null || true)"
[ -n "$ENVCLASS" ] || ENVCLASS="$(uname -m)-$(uname -s | tr 'A-Z' 'a-z')"
mkdir -p "$ROOT/sim/golden"
"$BUILD/cartpole_demo" --theta0 0.2 --x0 0.5 --seconds 1 > "$ROOT/sim/golden/demo-1s.$ENVCLASS.csv"
echo "wrote sim/golden/demo-1s.$ENVCLASS.csv"
H="$(python3 "$ROOT/sim/backtest.py" "$ROOT/sim/scenarios/repro.json" --hash --quiet)"
echo "$H" > "$ROOT/sim/golden/repro.$ENVCLASS.sha256"
echo "wrote sim/golden/repro.$ENVCLASS.sha256  ($H)"
