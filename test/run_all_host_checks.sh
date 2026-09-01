#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"

"$ROOT/test/run_native_tests.sh"

if command -v clang++ >/dev/null 2>&1; then
  CXX=clang++ "$ROOT/test/run_native_tests.sh"
fi

"$ROOT/test/run_sanitizers.sh"
"$ROOT/test/check_examples_host.sh"
"$ROOT/test/check_release_contracts.sh"
