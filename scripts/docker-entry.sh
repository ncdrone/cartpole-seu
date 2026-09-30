#!/usr/bin/env bash
# Container entrypoint. Commands:
#   verify                 scripts/run_tests.sh --quick: unit tests (controller, protect, fdir), golden, injector, campaign, repro hash (default)
#   tests [--quick]        scripts/run_tests.sh (adds the basin and mistune back-tests)
#   campaign [args...]     python3 sim/campaign.py <args...>
#   serve [--port N]       LAN viewer on 0.0.0.0:8080
#   backtest <args...>     python3 sim/backtest.py <args...>
#   demo <args...>         fsw/build/cartpole_demo <args...>   (CSV on stdout)
#   shell                  bash
set -eu
cd /work
case "${1:-verify}" in
  verify)  exec bash scripts/run_tests.sh --quick ;;
  tests)   shift; exec bash scripts/run_tests.sh "$@" ;;
  campaign) shift; exec python3 sim/campaign.py "$@" ;;
  serve)   shift; exec python3 bench/serve.py "$@" ;;
  backtest) shift; exec python3 sim/backtest.py "$@" ;;
  demo)    shift; exec ./fsw/build/cartpole_demo "$@" ;;
  shell)   shift; exec bash "$@" ;;
  *) echo "unknown command: $1 (verify | tests | campaign | serve | backtest | demo | shell)"; exit 2 ;;
esac
