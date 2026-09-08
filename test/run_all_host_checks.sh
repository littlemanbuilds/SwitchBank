#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# g++ can be an Apple Clang alias. Require genuine GNU predefines before testing.
is_gnu() {
  local defines
  defines="$("$1" -dM -E -x c++ /dev/null)" || return 1
  [[ "$defines" == *"#define __GNUC__ "* && "$defines" != *"#define __clang__ "* ]]
}

GCC="${GCC_CXX:-${GNU_CXX:-}}"
if [[ -z "$GCC" ]]; then
  while IFS= read -r candidate; do
    if is_gnu "$candidate"; then
      GCC="$candidate"
      break
    fi
  done < <(compgen -c | LC_ALL=C sort -u | grep -E '^g\+\+(-[0-9]+)?$')
fi
[[ -n "$GCC" ]] && is_gnu "$GCC" || { echo "FAIL: genuine GNU G++ required (set GCC_CXX or GNU_CXX)" >&2; exit 1; }
CLANG="${CLANG_CXX:-clang++}"
clang_defines="$("$CLANG" -dM -E -x c++ /dev/null)"
[[ "$clang_defines" == *"#define __clang__ "* ]] || { echo "FAIL: Clang required (set CLANG_CXX)" >&2; exit 1; }

"$GCC" --version
CXX="$GCC" bash "$ROOT/test/run_native_tests.sh"
"$CLANG" --version
CXX="$CLANG" bash "$ROOT/test/run_native_tests.sh"
CXX="$CLANG" bash "$ROOT/test/run_sanitizers.sh"
CXX="$CLANG" bash "$ROOT/test/check_examples_host.sh"
bash "$ROOT/test/check_release_contracts.sh"
