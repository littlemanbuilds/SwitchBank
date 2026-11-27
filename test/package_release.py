#!/usr/bin/env python3
"""Build and check SwitchBank distribution archives for an exact Git commit."""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import tarfile
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def git(repository, *args):
    return subprocess.check_output(
        ["git", "--no-optional-locks", "-C", str(repository), *args],
        env={**os.environ, "GIT_NO_REPLACE_OBJECTS": "1"},
    )


def inventories(repository):
    plan = json.loads((repository / "test/public_files.json").read_text())
    for paths in plan.values():
        if paths != sorted(set(paths)) or any(
            not re.fullmatch(r"[A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)*", path)
            or any(part in {"..", ".", ".lmb", ".agents", ".codex", ".pio", ".vscode",
                            ".git", ".DS_Store", "AGENTS.md"} for part in path.split("/"))
            for path in paths
        ):
            raise ValueError("Invalid positive distribution inventory")
    if set(plan) != {"github", "arduino", "platformio"}:
        raise ValueError("Distribution inventory must name all three channels")
    if plan["arduino"] != [path for path in plan["github"] if not path.startswith(".")]:
        raise ValueError("Arduino inventory differs from the public non-hidden Git projection")
    if not set(plan["platformio"]) <= set(plan["github"]):
        raise ValueError("PlatformIO inventory must be a subset of the public Git inventory")
    return plan


def git_entries(repository, commit):
    tree = {}
    for entry in git(repository, "ls-tree", "-rz", commit).decode().split("\0")[:-1]:
        metadata, name = entry.split("\t", 1)
        mode, kind, oid = metadata.split()
        if kind != "blob" or mode not in {"100644", "100755"}:
            raise ValueError(f"Unsupported release member: {name}")
        tree[name] = (mode, oid)
    index = {}
    for entry in git(repository, "ls-files", "--stage", "-z").decode().split("\0")[:-1]:
        metadata, name = entry.split("\t", 1)
        mode, oid, stage = metadata.split()
        if stage != "0":
            raise ValueError("Unmerged release index")
        index[name] = (mode, oid)
    if index != tree:
        raise ValueError("Release index differs from the selected commit")
    return tree


def verify_archive(archive, expected, prefix=""):
    """Check member type, path, completeness and bytes without extracting."""
    seen = set()

    def verify(name, regular, read):
        if not regular or not name.startswith(prefix):
            raise ValueError(f"Unsupported archive member: {name}")
        name = name[len(prefix):]
        if name not in expected or name in seen:
            raise ValueError(f"Unexpected or duplicate archive member: {name}")
        if read() != expected[name]:
            raise ValueError(f"Archive contents differ: {name}")
        seen.add(name)

    if archive.suffix == ".zip":
        with zipfile.ZipFile(archive) as package:
            for member in package.infolist():
                kind = stat.S_IFMT(member.external_attr >> 16)
                verify(member.filename, kind in {0, stat.S_IFREG} and not member.is_dir()
                       and member.orig_filename == member.filename and not member.flag_bits & 1,
                       lambda: package.read(member))
    else:
        with tarfile.open(archive, "r:gz") as package:
            for member in package:
                verify(member.name, member.isfile() and not member.sparse,
                       lambda: package.extractfile(member).read())
    if seen != expected.keys():
        raise ValueError(f"Missing archive members: {sorted(expected.keys() - seen)}")


def pack_platformio(pio, source, output, expected):
    manifest = json.loads(expected["library.json"])
    if manifest.get("export") != {"include": sorted(expected)}:
        raise ValueError("PlatformIO export rule differs from the positive inventory")
    subprocess.run([pio, "pkg", "pack", str(source), "--output", str(output)], check=True)
    verify_archive(output, expected)


def package_release(repository, commit, output, pio):
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise ValueError("Supply the full release commit SHA")
    if git(repository, "rev-parse", "--verify", f"{commit}^{{commit}}").decode().strip() != commit:
        raise ValueError("The release SHA must identify a commit")
    if output.resolve().is_relative_to(repository.resolve()):
        raise ValueError("Choose an archive output directory outside the source checkout")
    tree = git_entries(repository, commit)
    if "test/public_files.json" not in tree:
        raise ValueError("Selected commit has no public inventory")
    with tempfile.TemporaryDirectory(prefix="switchbank-release-") as temp:
        scratch = Path(temp)
        source = scratch / "source"
        plan_path = source / "test/public_files.json"
        plan_path.parent.mkdir(parents=True)
        plan_path.write_bytes(git(repository, "cat-file", "blob", tree["test/public_files.json"][1]))
        plan = inventories(source)
        if set(tree) != set(plan["github"]):
            raise ValueError("Committed tree differs from the positive GitHub inventory")
        expected = {}
        for name in plan["platformio"]:
            mode, oid = tree[name]
            expected[name] = git(repository, "cat-file", "blob", oid)
            path = source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(expected[name])
            path.chmod(int(mode, 8) & 0o777)
        version = json.loads(expected["library.json"])["version"]
        if not re.fullmatch(r"[0-9]+\.[0-9]+\.[0-9]+", version):
            raise ValueError("Expected a numeric release version")
        prefix = f"SwitchBank-{version}/"
        platformio = scratch / "platformio-raw.tar.gz"
        pack_platformio(pio, source, platformio, expected)
        for fmt in ("zip", "tar.gz"):
            (scratch / f"github.{fmt}").write_bytes(
                git(repository, "archive", f"--format={fmt}", f"--prefix={prefix}", commit)
            )
        arduino = scratch / "arduino.zip"
        arduino_bytes = {name: git(repository, "cat-file", "blob", tree[name][1])
                         for name in plan["arduino"]}
        with zipfile.ZipFile(arduino, "x", compression=zipfile.ZIP_DEFLATED) as archive:
            for name, content in arduino_bytes.items():
                entry = zipfile.ZipInfo(prefix + name)
                entry.external_attr = int(tree[name][0], 8) << 16
                archive.writestr(entry, content, compress_type=zipfile.ZIP_DEFLATED)
        verify_archive(arduino, arduino_bytes, prefix)
        output.mkdir()
        for name in ("github.zip", "github.tar.gz", arduino.name, platformio.name):
            shutil.copyfile(scratch / name, output / name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("commit")
    parser.add_argument("output", type=Path, help="New output directory outside the checkout")
    parser.add_argument("--pio", default="pio")
    args = parser.parse_args()
    package_release(ROOT, args.commit, args.output, args.pio)
