#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

fail() { echo "FAIL: $*" >&2; exit 1; }

version_h="$(sed -n 's/^#define SWITCHBANK_VERSION "\([^"]*\)"/\1/p' "$ROOT/src/SwitchBank.h")"
version_props="$(sed -n 's/^version=//p' "$ROOT/library.properties")"
version_json="$(python3 -B -c 'import json,sys; print(json.load(open(sys.argv[1]))["version"])' "$ROOT/library.json")"
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
  grep -Fq "$token" "$ROOT/README.md" || fail "README missing API contract: $token"
done

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
done < <(if [[ -d "$ROOT/.git" ]]; then git -C "$ROOT" ls-files -z; fi)
if "$tracked_debris"; then
  fail "tracked package debris found in release content"
fi

python3 -B -c '
import pathlib, re, sys
root = pathlib.Path(sys.argv[1])
version = sys.argv[2]
header = (root / "src/SwitchBank.h").read_text()
readme = (root / "README.md").read_text()
tail = readme.split("## Version history", 1)[1]
assert "```text\n" + version + "\n```" in tail, "README current version disagrees"
assert "See [CHANGELOG.md](CHANGELOG.md) for detailed release history." in tail, "missing history link"
numeric = [re.search(r"^#define SWITCHBANK_VERSION_" + part + r" (\d+)$", header, re.M).group(1)
           for part in ("MAJOR", "MINOR", "PATCH")]
assert ".".join(numeric) == version, "numeric version declarations disagree"
changelog = (root / "CHANGELOG.md").read_text()
assert re.search(r"^## \[" + re.escape(version) + r"\] - \d{4}-\d{2}-\d{2}$", changelog, re.M), "missing current changelog"
assert "## [1.2.0] - 2026-09-01" in changelog and "## [1.1.0]" in changelog, "missing release history"
' "$ROOT" "$version_h"

echo "PASS: release contracts"
