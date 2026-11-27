#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="$(mktemp -d "${TMPDIR:-/tmp}/switchbank-native.XXXXXX")"
trap 'rm -rf "$BUILD"' EXIT
CXX="${CXX:-c++}"
"$CXX" -std=c++11 -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow -Werror \
  -I"$ROOT/src" "$ROOT/test/native/test_switchbank.cpp" -o "$BUILD/test_switchbank"
"$BUILD/test_switchbank"
