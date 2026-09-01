#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

fail() { echo "FAIL: $*" >&2; exit 1; }

version_h="$(sed -n 's/^#define SWITCHBANK_VERSION "\([^"]*\)"/\1/p' "$ROOT/src/SwitchBank.h")"
version_props="$(sed -n 's/^version=//p' "$ROOT/library.properties")"
version_json="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["version"])' "$ROOT/library.json")"
[[ "$version_h" == "$version_props" && "$version_h" == "$version_json" ]] || fail "version metadata is inconsistent"
! grep -Eq '^#define[[:space:]]+LIBRARY_VERSION([_[:space:]])' "$ROOT/src/SwitchBank.h" || fail "generic LIBRARY_VERSION aliases must remain absent"

if [[ -d "$ROOT/.git" ]] && git -C "$ROOT" check-ignore -q test/native/test_switchbank.cpp; then
  fail ".gitignore must not ignore committed test sources"
fi

for heading in \
  '## Contents' \
  '## Installation' \
  '## Beginner path' \
  '## Testing' \
  '## Migration from v1.1' \
  '## Limitations'; do
  grep -Fq "$heading" "$ROOT/README.md" || fail "README missing: $heading"
done

for token in \
  'SwitchBankReadResult' \
  'SwitchBankPackedReadResult' \
  'sampleMs()' \
  'changeSequence()' \
  'generation()' \
  'consumeEdges()' \
  'forceCommitForCommissioning()'; do
  grep -Fq "$token" "$ROOT/README.md" || fail "README missing audit remediation: $token"
done

grep -Fq 'Current version: **1.2.0**. See [CHANGELOG.md](CHANGELOG.md)' "$ROOT/README.md" ||
  fail "README version-history tail is inconsistent"
grep -Fq 'SwitchBank is released under the **MIT License**. See [LICENSE](LICENSE).' "$ROOT/README.md" ||
  fail "README license tail is inconsistent"
grep -Fq 'Copyright © 2026 Little Man Builds (Darren Osborne).' "$ROOT/README.md" ||
  fail "README copyright identity is inconsistent"

for f in "$ROOT"/src/*.h "$ROOT"/examples/*/*.ino; do
  grep -Fq '@file' "$f" || fail "missing @file Doxygen header: $f"
  grep -Fq '@author Little Man Builds (Darren Osborne)' "$f" || fail "missing LMB author header: $f"
done

for sketch in \
  examples/01_SimpleDIP3Bit/01_SimpleDIP3Bit.ino \
  examples/02_ModeNames3Bit/02_ModeNames3Bit.ino \
  examples/03_PortExpander8Bit/03_PortExpander8Bit.ino \
  examples/04_PortExpanderCachedRead/04_PortExpanderCachedRead.ino; do
  [[ -f "$ROOT/$sketch" ]] || fail "missing public example: $sketch"
done

[[ -f "$ROOT/.github/workflows/ci.yml" ]] || fail "missing GitHub Actions CI workflow"
[[ ! -e "$ROOT/platformio.ci.ini" ]] || fail "legacy platformio.ci.ini must not ship"
tracked_debris=false
while IFS= read -r -d '' path; do
  [[ -e "$ROOT/$path" ]] || continue
  [[ "$path" == ".vscode/extensions.json" ]] && continue
  if [[ "$path" =~ (^|/)(\.DS_Store|__MACOSX(/|$)|\.pio/|\.vscode/|build/|dist/) ]] ||
     [[ "$path" =~ \.(zip|ZIP|o|obj|elf|bin|hex|map)$ ]]; then
    tracked_debris=true
    break
  fi
done < <(git -C "$ROOT" ls-files -z)
if "$tracked_debris"; then
  fail "tracked package debris found in release content"
fi

echo "PASS: release contracts"
