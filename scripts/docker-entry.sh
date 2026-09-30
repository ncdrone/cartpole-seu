#!/usr/bin/env bash
# Container entrypoint. Commands:
#   verify                 re-run unit tests + golden check + print the repro hash (default)
#   serve [--port N]       LAN viewer on 0.0.0.0:8080
#   backtest <args...>     python3 sim/backtest.py <args...>
#   demo <args...>         fsw/build/cartpole_demo <args...>   (CSV on stdout)
#   shell                  bash
set -eu
cd /work
case "${1:-verify}" in
  verify)
    ./fsw/build/test_controller && ./fsw/build/test_protect | tail -1
    echo "envclass: $(python3 -c 'import json;print(json.load(open("fsw/build/build_info.json"))["envclass"])')"
    python3 sim/backtest.py sim/scenarios/repro.json --hash
    ;;
  tests)   shift; exec bash scripts/run_tests.sh "$@" ;;
  campaign) shift; exec python3 sim/campaign.py "$@" ;;
  serve)   shift; exec python3 bench/serve.py "$@" ;;
  backtest) shift; exec python3 sim/backtest.py "$@" ;;
  demo)    shift; exec ./fsw/build/cartpole_demo "$@" ;;
  shell)   shift; exec bash "$@" ;;
  *) echo "unknown command: $1 (verify | tests | campaign | serve | backtest | demo | shell)"; exit 2 ;;
esac
