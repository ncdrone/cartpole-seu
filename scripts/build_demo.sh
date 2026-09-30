#!/usr/bin/env bash
# build_demo.sh: build and verify the cart-pole controller on metal (macOS, Raspberry Pi OS, Jetson L4T, Debian/Ubuntu).
#   bash scripts/build_demo.sh                  detect -> deps (offers installs, y/N) -> build -> verify -> stamp. Pure C++.
#   --force-default   every y/N is yes            --no-install   print install commands, never run them
#   --visual          matplotlib window after     --serve        LAN viewer on :8080 after (bench/serve.py)
#   --demo            host-aware: Mac window, Pi/Jetson LAN viewer
#   --docker          also build the pinned image and verify inside it (Docker is never installed by this script)
#   --clean           wipe fsw/build first        --explain      what is checked and why
# Do NOT run with sudo: it builds as you and prefixes sudo only on apt-get lines. Run as root anyway: warns, drops sudo.
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"; FSW="$ROOT/fsw"; BUILD="$FSW/build"
VISUAL=0; SERVE=0; DOCKER=0; DEMO=0; INSTALL=1; YES=0; CLEAN=0
for a in "$@"; do case "$a" in
  --visual) VISUAL=1 ;; --serve) SERVE=1 ;; --demo) DEMO=1 ;; --docker) DOCKER=1 ;; --clean) CLEAN=1 ;;
  --force-default|--yes|-y) YES=1 ;; --no-install) INSTALL=0 ;; --install-deps) INSTALL=1 ;;
  --explain) sed -n '/^# EXPLAIN$/,/^# END$/p' "$0" | sed '1d;$d;s/^# \{0,1\}//'; exit 0 ;;
  -h|--help) sed -n 2,9p "$0"; exit 0 ;; *) echo "unknown flag: $a (see --help)"; exit 2 ;; esac; done
say(){ printf '\033[1m%s\033[0m\n' "$*"; }; ok(){ printf '  \033[32mok\033[0m   %s\n' "$*"; }
warn(){ printf '  \033[33mwarn\033[0m %s\n' "$*"; }; miss(){ printf '  \033[31mmissing\033[0m %s\n' "$*"; }
die(){ printf '  \033[31mFAIL\033[0m %s\n' "$*"; exit 1; }
# EXPLAIN
# 1 detect   OS, arch, board (Pi / Jetson from the device tree), glibc. Environment class = arch-os (arm64-darwin,
#            aarch64-linux); goldens are keyed by it. glibc 2.35/2.36/2.41 were measured to agree.
# 2 deps     compiler: compile AND run a 3-line file (a clean Mac has a clang++ shim that only prompts for the CLT);
#            cmake; python3 + matplotlib only when --visual/--serve/--demo need them; docker only with --docker.
#            Missing items are offered y/N (default no); --force-default = yes to all; --no-install = print only.
#            apt-get lines carry sudo (prompts once); brew/pip do not. Homebrew and Docker are never installed.
# 3 build    Release, -std=c++14 -O2 -ffp-contract=off -fno-exceptions -fno-rtti, warnings are errors.
# 4 stamp    fsw/build/build_info.json: host, arch, envclass, compiler, flags, libc, git SHA, time, container.
# 5 verify   unit tests; 1 s demo with CSV schema check; golden: bit-exact for this class, else per-column delta.
# 6 extras   --visual / --serve / --demo / --docker as listed above. Docker is never the default.
# END
ASROOT=0; [ "$(id -u)" = 0 ] && { ASROOT=1; warn "running as root: not required; the build tree will be root-owned"; }
INSTALLED=0
offer(){ # offer "<command>"
  local cmd="$1"; [ "$ASROOT" = 1 ] && cmd="${cmd//sudo /}"
  echo "         install: $cmd"; [ "$INSTALL" = 1 ] || return 0
  local ans=n
  if [ "$YES" = 1 ]; then ans=y; elif [ -r /dev/tty ]; then printf '         run this now? [y/N] '; read -r ans < /dev/tty || ans=n; else echo "         (no tty; use --force-default)"; fi
  case "$ans" in y|Y|yes|YES) say "  running: $cmd"; bash -c "$cmd" || die "install failed"; INSTALLED=1 ;; *) echo "         skipped" ;; esac
}

say "1/6 detect"
OS="$(uname -s)"; ARCH="$(uname -m)"; HOST="linux"; MODEL=""; LIBC=""; PKG=""
case "$OS" in
  Darwin) HOST=macos; PKG=brew ;;
  Linux) [ -r /proc/device-tree/model ] && MODEL="$(tr -d '\0' < /proc/device-tree/model)"
         if [ -r /etc/nv_tegra_release ] || echo "$MODEL" | grep -qi nvidia; then HOST=jetson; elif echo "$MODEL" | grep -qi raspberry; then HOST=raspberry-pi; fi
         command -v apt-get >/dev/null 2>&1 && PKG=apt; command -v ldd >/dev/null 2>&1 && LIBC="$(ldd --version 2>&1 | head -1 | awk '{print $NF}')" ;;
  *) die "unsupported OS: $OS" ;;
esac
ENVCLASS="${ARCH}-$(echo "$OS" | tr 'A-Z' 'a-z')"
ok "host=$HOST arch=$ARCH ${MODEL:+model=\"$MODEL\" }envclass=$ENVCLASS"
if [ "$DEMO" = 1 ]; then if [ "$HOST" = macos ]; then VISUAL=1; ok "--demo: window after build"; else SERVE=1; ok "--demo: LAN viewer after build"; fi; fi

say "2/6 dependencies$( [ "$INSTALL" = 0 ] && echo ' (--no-install)' || { [ "$YES" = 1 ] && echo ' (--force-default)' || echo ' (missing items offered y/N)'; })"
MISSING=0; T="$(mktemp -d 2>/dev/null || mktemp -d -t bd)"; printf '#include <cmath>\nint main(){return std::isfinite(1.0f)?0:1;}\n' > "$T/t.cpp"
CXX=""; for c in c++ g++ clang++; do command -v "$c" >/dev/null 2>&1 && "$c" -std=c++14 "$T/t.cpp" -o "$T/t" >/dev/null 2>&1 && "$T/t" && { CXX="$c"; break; }; done; rm -rf "$T"
CXXV=""; if [ -n "$CXX" ]; then CXXV="$("$CXX" --version | head -1)"; ok "compiler: $CXX ($CXXV)"; else MISSING=1; miss "C++14 compiler"
  [ "$HOST" = macos ] && offer "xcode-select --install" || offer "sudo apt-get update && sudo apt-get install -y build-essential g++"; fi
if command -v cmake >/dev/null 2>&1; then ok "cmake: $(cmake --version | head -1)"; else MISSING=1; miss "cmake"
  if [ "$HOST" = macos ]; then command -v brew >/dev/null 2>&1 && offer "brew install cmake" || echo "         install: Homebrew first (https://brew.sh), then brew install cmake"; else offer "sudo apt-get update && sudo apt-get install -y cmake"; fi; fi
HAVE_PY=0; HAVE_MPL=0
if command -v python3 >/dev/null 2>&1; then HAVE_PY=1; ok "python3: $(python3 --version 2>&1) (optional)"; python3 -c "import matplotlib" >/dev/null 2>&1 && { HAVE_MPL=1; ok "matplotlib (window viewer, PNG maps)"; } || warn "matplotlib absent (window viewer, PNG maps unavailable)"
elif [ "$VISUAL" = 1 ] || [ "$SERVE" = 1 ]; then MISSING=1; miss "python3 (for --visual/--serve)"; [ "$HOST" = macos ] && offer "brew install python" || offer "sudo apt-get install -y python3"
else warn "python3 absent: pure C++ path only"; fi
if [ "$VISUAL" = 1 ] && [ "$HAVE_PY" = 1 ] && [ "$HAVE_MPL" = 0 ]; then MISSING=1; miss "matplotlib (for --visual)"; [ "$PKG" = apt ] && offer "sudo apt-get install -y python3-matplotlib" || offer "python3 -m pip install --user matplotlib"; fi
if [ "$DOCKER" = 1 ]; then command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1 && ok "docker: $(docker --version)" || { MISSING=1; miss "docker (not installed or daemon down); never installed by this script"; echo "         install: https://docs.docker.com/get-started/get-docker/"; }; fi
if [ "$MISSING" != 0 ]; then [ "$INSTALLED" = 1 ] && { say "installed; re-checking"; exec bash "$0" "$@" --no-install; }; die "install the items above, then re-run (default offers them; --force-default auto-confirms)"; fi

say "3/6 build (Release, C++14, -O2 -ffp-contract=off, warnings are errors)"
[ "$CLEAN" = 1 ] && rm -rf "$BUILD"
cmake -S "$FSW" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release >/dev/null
if cmake --build "$BUILD" 2>&1 | grep -E "error|warning"; then die "build produced warnings/errors above"; fi
ok "fsw/build/cartpole_demo (protected), cartpole_baseline, test_controller, test_protect, test_fdir"

say "4/6 stamp"
esc(){ printf '%s' "$1" | sed 's/\\/\\\\/g; s/"/\\"/g'; }
cat > "$BUILD/build_info.json" <<JSON
{ "host": "$HOST", "os": "$OS", "arch": "$ARCH", "model": "$(esc "$MODEL")", "envclass": "$ENVCLASS",
  "compiler": "$CXX", "compiler_version": "$(esc "$CXXV")", "libc": "$LIBC",
  "flags": "-std=c++14 -O2 -ffp-contract=off -fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic -Werror",
  "git": "$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo no-git)", "built_utc": "$(date -u +%Y-%m-%dT%H:%M:%SZ)", "container": false }
JSON
ok "fsw/build/build_info.json ($ENVCLASS)"

say "5/6 verify"
"$BUILD/test_controller" >/dev/null && "$BUILD/test_protect" >/dev/null && "$BUILD/test_fdir" >/dev/null || die "unit tests failed"; ok "unit tests (controller + protect + fdir)"
CSV="$BUILD/verify-demo-1s.csv"; "$BUILD/cartpole_demo" --theta0 0.2 --x0 0.5 --seconds 1 > "$CSV" || die "demo failed"
[ "$(head -1 "$CSV")" = "tick,t,x,xdot,theta,thetadot,u,fault,u_x,u_xd,u_th,u_thd,u_i,integ,sat,mode,det,fdir,sec,reload" ] && [ "$(wc -l < "$CSV" | tr -d ' ')" = 101 ] || die "CSV schema"
ok "demo runs, CSV schema intact"
G="$ROOT/sim/golden/demo-1s.$ENVCLASS.csv"
if [ -f "$G" ]; then cmp -s "$G" "$CSV" && ok "bit-exact match with golden demo-1s.$ENVCLASS.csv" || die "differs from golden for $ENVCLASS: the same software constraint must reproduce bit-exact"
else warn "no golden for $ENVCLASS (scripts/make_golden.sh records one)"
  [ "$HAVE_PY" = 1 ] && python3 - "$CSV" "$ROOT/sim/golden" <<'EOF'
import csv, glob, os, sys
rows = list(csv.DictReader(open(sys.argv[1])))
for g in sorted(glob.glob(os.path.join(sys.argv[2], "demo-1s.*.csv"))):
    ref = list(csv.DictReader(open(g))); w = {k: max(abs(float(a[k]) - float(b[k])) for a, b in zip(rows, ref)) for k in ("x", "xdot", "theta", "thetadot", "u")}
    m = max(w.values()); print(f"  {'ok  ' if m < 1e-4 else 'warn'} vs golden {os.path.basename(g)[8:-4]}: max |Δ| " + " ".join(f"{k}={v:.1e}" for k, v in w.items()) + ("  (cross-libm drift, expected)" if 0 < m < 1e-4 else ""))
EOF
fi

say "6/6 ready"
echo "  run:     fsw/build/cartpole_demo --theta0 0.3 --x0 0.5 --seconds 8 > run.csv      (pure C++, CSV out)"
echo "  inject:  fsw/build/cartpole_baseline --theta0 0.05 --seconds 6 --flip k2:31 --flip-tick 100   (protected: fsw/build/cartpole_demo, same flags)"
echo "  tests:   bash scripts/run_tests.sh        visual: --visual | --demo | --serve        container: --docker"
if [ "$DOCKER" = 1 ]; then say "docker"; docker build -q -t cartpole "$ROOT" >/dev/null && ok "image cartpole" || die "docker build failed"; docker run --rm cartpole || die "container verify failed"; fi
if [ "$VISUAL" = 1 ]; then [ "$HAVE_MPL" = 1 ] || die "--visual needs python3 + matplotlib"; python3 "$ROOT/sim/render.py" --theta0 0.6 --x0 -0.5 --seconds 8; fi
if [ "$SERVE" = 1 ]; then [ "$HAVE_PY" = 1 ] || die "--serve needs python3"
  IP=""; if [ "$OS" = Darwin ]; then IP="$(ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null || true)"; else IP="$(hostname -I 2>/dev/null | awk '{print $1}')"; fi
  say "LAN viewer: http://${IP:-<this-host>}:8080   (ctrl-c to stop)"; exec python3 "$ROOT/bench/serve.py" --port 8080; fi
