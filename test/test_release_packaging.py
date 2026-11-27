#!/usr/bin/env python3
"""Exercise the exact-commit and PlatformIO boundaries using disposable data."""

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import unittest
import zipfile

from package_release import ROOT, git, inventories, package_release, pack_platformio, verify_archive


class ReleasePackaging(unittest.TestCase):
    def test_exact_commit_and_live_export(self):
        plan = inventories(ROOT)
        self.assertEqual(json.loads((ROOT / "library.json").read_text())["export"],
                         {"include": plan["platformio"]})
        with tempfile.TemporaryDirectory(prefix="switchbank-package-test-") as temp:
            scratch = Path(temp)
            repo = scratch / "repo"
            repo.mkdir()
            git(repo, "init", "-q")
            git(repo, "config", "user.name", "SwitchBank test")
            git(repo, "config", "user.email", "switchbank@example.invalid")
            for name in plan["github"]:
                target = repo / name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(ROOT / name, target)
            git(repo, "add", "--", *plan["github"])
            git(repo, "commit", "-qm", "Public fixture")
            commit = git(repo, "rev-parse", "HEAD").decode().strip()

            ignored = repo / ".vscode/local.txt"
            ignored.parent.mkdir()
            ignored.write_text("ignored private sentinel")
            untracked = repo / "src/private.txt"
            untracked.write_text("untracked private sentinel")
            expected = {name: (repo / name).read_bytes() for name in plan["platformio"]}
            manifest = repo / "library.json"
            original = json.loads(expected["library.json"])
            del original["export"]
            manifest.write_text(json.dumps(original))
            defective = scratch / "without-export.tar.gz"
            subprocess.run([PIO, "pkg", "pack", str(repo), "--output", str(defective)], check=True)
            with tarfile.open(defective) as archive:
                self.assertIn("src/private.txt", archive.getnames())
            manifest.write_bytes(expected["library.json"])
            pack_platformio(PIO, repo, scratch / "live.tar.gz", expected)

            (repo / "README.md").write_text("working tree differs from the committed public README")
            output = scratch / "release"
            package_release(repo, commit, output, PIO)
            verify_archive(output / "platformio-raw.tar.gz", expected)
            with tarfile.open(output / "platformio-raw.tar.gz") as archive:
                self.assertEqual(archive.extractfile("README.md").read(), expected["README.md"])

            git(repo, "add", "README.md")
            with self.assertRaisesRegex(ValueError, "index differs"):
                package_release(repo, commit, scratch / "changed-index", PIO)
            git(repo, "restore", "--staged", "README.md")
            git(repo, "add", "src/private.txt")
            untracked.write_text("different from the staged private sentinel")
            with self.assertRaisesRegex(ValueError, "index differs"):
                package_release(repo, commit, scratch / "staged", PIO)
            git(repo, "commit", "-qm", "Unauthorized fixture")
            dirty_commit = git(repo, "rev-parse", "HEAD").decode().strip()
            with self.assertRaisesRegex(ValueError, "differs from the positive GitHub inventory"):
                package_release(repo, dirty_commit, scratch / "tracked", PIO)

    def test_archive_mutations(self):
        with tempfile.TemporaryDirectory(prefix="switchbank-archive-test-") as temp:
            cases = [
                ({"pkg/README.md": b"readme", "pkg/more.txt": b"extra"}, "Unexpected"),
                ({}, "Missing"),
                ({"pkg/README.md": b"modified"}, "contents differ"),
                ({"pkg/../README.md": b"readme"}, "Unexpected"),
                ({"other/README.md": b"readme"}, "Unsupported"),
            ]
            for files, error in cases:
                with self.subTest(error=error, files=files):
                    archive = Path(temp) / "bad.zip"
                    with zipfile.ZipFile(archive, "w") as package:
                        for name, data in files.items():
                            package.writestr(name, data)
                    with self.assertRaisesRegex(ValueError, error):
                        verify_archive(archive, {"README.md": b"readme"}, "pkg/")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pio", default="pio")
    args = parser.parse_args()
    PIO = args.pio
    unittest.main(argv=[__file__])
