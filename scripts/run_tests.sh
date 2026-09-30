#!/usr/bin/env bash
# run_tests.sh: everything that can fail, in order, with a non-zero exit on the first failure.
#
#   bash scripts/run_tests.sh            unit tests -> demo schema -> golden (bit-exact for this envclass) -> repro hash
#                                        -> back-test scenarios (python3; skipped with a note if absent)
#   bash scripts/run_tests.sh --quick    skip the back-test scenarios
#   bash scripts/run_tests.sh --docker   run the same inside the pinned container (builds the image if needed)
#
# Requires a prior `bash scripts/build_demo.sh`. Never installs anything, never needs sudo.
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/fsw/build"
QUICK=0; DOCKER=0
for a in "$@"; do case "$a" in --quick) QUICK=1 ;; --docker) DOCKER=1 ;; -h|--help) sed -n 2,9p "$0"; exit 0 ;; *) echo "unknown flag: $a"; exit 2 ;; esac; done
say(){ printf '\033[1m%s\033[0m\n' "$*"; }; ok(){ printf '  \033[32mok\033[0m   %s\n' "$*"; }; warn(){ printf '  \033[33mwarn\033[0m %s\n' "$*"; }; die(){ printf '  \033[31mFAIL\033[0m %s\n' "$*"; exit 1; }

if [ "$DOCKER" -eq 1 ]; then
  command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1 || die "docker not available"
  docker image inspect cartpole >/dev/null 2>&1 || { say "building image"; docker build -t cartpole "$ROOT" >/dev/null; }
  exec docker run --rm cartpole tests $( [ "$QUICK" -eq 1 ] && echo --quick )
fi

[ -x "$BUILD/test_controller" ] && [ -x "$BUILD/cartpole_demo" ] || die "not built: run bash scripts/build_demo.sh first"
ENVCLASS="$(sed -n 's/.*"envclass": "\([^"]*\)".*/\1/p' "$BUILD/build_info.json" 2>/dev/null || true)"
[ -n "$ENVCLASS" ] || ENVCLASS="$(uname -m)-$(uname -s | tr 'A-Z' 'a-z')"
say "tests for envclass $ENVCLASS"

say "1/6 unit tests"
"$BUILD/test_controller" | tail -1 | grep -q "ALL TESTS PASSED" || die "test_controller failed"; ok "test_controller"
"$BUILD/test_protect" | tail -1 | grep -q "ALL PROTECT TESTS PASSED" || die "test_protect failed"; ok "test_protect (exhaustive SECDED 1/2/3-bit, CRC-32C, store)"

say "2/6 demo schema"
OUT="$("$BUILD/cartpole_demo" --theta0 0.2 --x0 0.5 --seconds 1)" || die "demo exited non-zero"
[ "$(printf '%s\n' "$OUT" | head -1)" = "tick,t,x,xdot,theta,thetadot,u,fault,u_x,u_xd,u_th,u_thd,u_i,integ,sat,mode,det,fdir,sec,reload" ] || die "CSV header changed"
[ "$(printf '%s\n' "$OUT" | wc -l | tr -d ' ')" = "101" ] || die "row count"
printf '%s\n' "$OUT" | awk -F, 'NR>1 && $8!=0 {bad=1} END {exit bad}' || die "fault flag raised on a nominal run"
ok "header, row count, no faults"

say "3/6 golden (bit-exact for this envclass)"
G="$ROOT/sim/golden/demo-1s.$ENVCLASS.csv"
if [ -f "$G" ]; then printf '%s\n' "$OUT" | cmp -s - "$G" && ok "matches $(basename "$G")" || die "demo differs from $(basename "$G")"; else warn "no golden for $ENVCLASS (scripts/make_golden.sh records one)"; fi

say "4/6 injector sanity: the same sign flip must drop the pole on the baseline and be corrected on the protected build"
"$BUILD/cartpole_baseline" --theta0 0.05 --seconds 4 --flip k2:31 --flip-tick 100 2>/dev/null | awk -F, 'NR>1 {t=$5; if (t<0) t=-t; if (t>1.57) fell=1} END {exit !fell}' && ok "baseline: k2:31 drops the pole" || die "baseline did not fall on k2:31: injector or plant changed"
"$BUILD/cartpole_demo" --theta0 0.05 --seconds 4 --flip k2:31 --flip-tick 100 2>/dev/null | awk -F, 'NR>1 {t=$5; if (t<0) t=-t; if (t>0.087) bad=1; if ($1==100 && $17!=1) bad=1} END {exit bad}' && ok "protected: k2:31 corrected at tick 100 (det=SEC), pole unaffected" || die "protected build did not correct k2:31"
"$BUILD/cartpole_demo" --theta0 0.05 --seconds 4 --flip k2:30+31 --flip-tick 100 2>/dev/null | awk -F, 'NR>1 {t=$5; if (t<0) t=-t; if (t>0.087) bad=1; if ($1==100 && $17!=2) bad=1; if ($1==399 && $20<1) bad=1} END {exit bad}' && ok "protected: k2:30+31 -> DED -> verified golden reload, pole unaffected" || die "protected build did not recover from a double flip"

say "5/6 verification campaign (every word, every bit, both builds)"
if command -v python3 >/dev/null 2>&1; then python3 "$ROOT/sim/campaign.py" $( [ "$QUICK" -eq 1 ] && echo --quick ) 2>&1 | grep -E "^== |protected  |baseline  |thresholds|FAIL" | sed 's/^/   /'; [ "${PIPESTATUS[0]}" -eq 0 ] || die "campaign thresholds not met"; ok "campaign thresholds met"; else warn "python3 absent: campaign skipped"; fi

say "6/6 back-tests + repro hash"
if command -v python3 >/dev/null 2>&1; then
  H="$(python3 "$ROOT/sim/backtest.py" "$ROOT/sim/scenarios/repro.json" --quiet)"
  GH="$ROOT/sim/golden/repro.$ENVCLASS.sha256"
  if [ -f "$GH" ]; then [ "$H" = "$(cat "$GH")" ] && ok "repro hash matches golden ($H)" || die "repro hash $H != golden $(cat "$GH")"; else warn "repro hash $H (no golden for $ENVCLASS)"; fi
  if [ "$QUICK" -eq 0 ]; then
    python3 "$ROOT/sim/backtest.py" "$ROOT/sim/scenarios/basin.json" "$ROOT/sim/scenarios/mistune.json" 2>&1 | grep -E "^==|success|basin  " | sed 's/^/   /'
    ok "scenarios ran (stats above; CSVs in sim/out/)"
  fi
else
  warn "python3 absent: back-tests and repro hash skipped (pure C++ checks 1-4 passed)"
fi
say "all tests passed"
