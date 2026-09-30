#!/usr/bin/env bash
# clean_up.sh: remove everything the demo generated. Goldens, sources and docs are never touched.
#
#   bash scripts/clean_up.sh                 build trees, back-test outputs, verify CSVs, caches, stop viewer processes
#   bash scripts/clean_up.sh --docker        ...also remove the cartpole Docker image (asks y/N)
#   bash scripts/clean_up.sh --remote        ...also delete ~/orca-cartpole on the boards named in REMOTE_HOSTS (asks y/N each)
#   bash scripts/clean_up.sh --all           --docker --remote
#   bash scripts/clean_up.sh --force-default answer yes to every y/N
#   bash scripts/clean_up.sh --dry-run       print what would be removed and exit
#
# Never needs sudo. Safe to run twice.
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DOCKER=0; REMOTE=0; YES=0; DRY=0
REMOTE_HOSTS="${REMOTE_HOSTS:-frame orion}"
for a in "$@"; do
  case "$a" in
    --docker) DOCKER=1 ;; --remote) REMOTE=1 ;; --all) DOCKER=1; REMOTE=1 ;;
    --force-default|--yes|-y) YES=1 ;; --dry-run) DRY=1 ;;
    -h|--help) sed -n 2,11p "$0"; exit 0 ;;
    *) echo "unknown flag: $a"; exit 2 ;;
  esac
done
say(){ printf '\033[1m%s\033[0m\n' "$*"; }; ok(){ printf '  \033[32mok\033[0m   %s\n' "$*"; }; skip(){ printf '  \033[90m--\033[0m   %s\n' "$*"; }
confirm() {  # confirm "<question>" -> 0 on yes
  [ "$YES" -eq 1 ] && return 0
  printf '  %s [y/N] ' "$1"; if [ -r /dev/tty ]; then read -r ans < /dev/tty || ans=n; else ans=n; echo "(no tty)"; fi
  case "$ans" in y|Y|yes|YES) return 0 ;; *) return 1 ;; esac
}
rm_path() {  # rm_path <relative path>
  if [ -e "$ROOT/$1" ]; then
    if [ "$DRY" -eq 1 ]; then echo "  would remove $1"; else rm -rf "${ROOT:?}/$1"; ok "removed $1"; fi
  else skip "$1 (absent)"; fi
}

say "1/4 stop viewer processes"
for pat in "bench/serve.py" "sim/render.py"; do
  if pgrep -f "$pat" >/dev/null 2>&1; then
    if [ "$DRY" -eq 1 ]; then echo "  would stop $pat ($(pgrep -f "$pat" | tr '\n' ' '))"; else pkill -f "$pat" && ok "stopped $pat"; fi
  else skip "$pat (not running)"; fi
done

say "2/4 generated files"
rm_path fsw/build
rm_path fsw/build-xcode
rm_path sim/out
rm_path sim/__pycache__
find "$ROOT" -name "__pycache__" -type d -not -path "*/fsw/build/*" 2>/dev/null | while read -r d; do rel="${d#$ROOT/}"; rm_path "$rel"; done
find "$ROOT" \( -name "*.pyc" -o -name ".DS_Store" \) -not -path "*/fsw/build/*" 2>/dev/null | while read -r f; do rel="${f#$ROOT/}"; rm_path "$rel"; done
skip "sim/golden/ kept (reference results; delete by hand if you mean to re-baseline)"

say "3/4 docker image"
if [ "$DOCKER" -eq 1 ]; then
  if command -v docker >/dev/null 2>&1 && [ -n "$(docker images -q cartpole 2>/dev/null)" ]; then
    if [ "$DRY" -eq 1 ]; then echo "  would remove docker image cartpole (and cartpole:amd64 if present)"
    elif confirm "remove docker image 'cartpole'?"; then docker rmi -f cartpole >/dev/null; docker rmi -f cartpole:amd64 >/dev/null 2>&1 || true; ok "docker image removed"
    else skip "docker image kept"; fi
  else skip "no cartpole image (or no docker)"; fi
else skip "not requested (--docker)"; fi

say "4/4 remote copies on boards"
if [ "$REMOTE" -eq 1 ]; then
  for h in $REMOTE_HOSTS; do
    if [ "$DRY" -eq 1 ]; then echo "  would run on $h: rm -rf ~/orca-cartpole"
    elif confirm "delete ~/orca-cartpole on $h?"; then
      ssh -o BatchMode=yes -o ConnectTimeout=5 "$h" 'pkill -f bench/serve.py 2>/dev/null; rm -rf ~/orca-cartpole' && ok "$h cleaned" || skip "$h unreachable"
    else skip "$h kept"; fi
  done
else skip "not requested (--remote)"; fi
say "done"
