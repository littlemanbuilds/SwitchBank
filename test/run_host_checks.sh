#!/bin/sh
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# Explicitly select Bash even when a caller invokes this entry point with sh.
exec bash "$ROOT/test/run_all_host_checks.sh" "$@"
