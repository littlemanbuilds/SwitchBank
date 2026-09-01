#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$ROOT/test/build"
CXX="${CXX:-c++}"
mkdir -p "$BUILD"
"$CXX" -std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror \
  -fsanitize=address,undefined -fno-omit-frame-pointer -I"$ROOT/src" \
  "$ROOT/test/native/test_switchbank.cpp" -o "$BUILD/test_switchbank_san"
ASAN_OPTIONS="${ASAN_OPTIONS:-detect_leaks=0}" "$BUILD/test_switchbank_san"
