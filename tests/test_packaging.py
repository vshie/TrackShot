"""packaging/macho_minos.py: the bundle's LSMinimumSystemVersion comes from its binaries."""

import importlib.util
import plistlib
import shutil
import sys
from pathlib import Path

import pytest

if sys.platform != "darwin":
    pytest.skip("Mach-O binaries are macOS only", allow_module_level=True)

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("macho_minos", ROOT / "packaging" / "macho_minos.py")
macho_minos = importlib.util.module_from_spec(spec)
spec.loader.exec_module(macho_minos)

def interpreter_binary() -> Path:
    return Path(sys.executable).resolve()


def test_minos_of_a_real_binary_and_of_other_files(tmp_path):
    version = macho_minos.minos(interpreter_binary())
    assert version is not None and version >= (11, 0, 0)
    text_file = tmp_path / "notes.txt"
    text_file.write_text("not a binary")
    assert macho_minos.minos(text_file) is None


def test_required_macos_is_the_highest_minos_with_a_floor():
    binary = interpreter_binary()
    assert macho_minos.required_macos([binary]) == macho_minos.as_text(max(macho_minos.minos(binary), (11, 0, 0)))
    assert macho_minos.required_macos([]) == "11.0"
    assert macho_minos.parse("14.4") == (14, 4, 0) and macho_minos.as_text((27, 0, 0)) == "27.0"


def make_bundle(tmp_path, declared: str) -> Path:
    app = tmp_path / "Test.app"
    (app / "Contents" / "MacOS").mkdir(parents=True)
    shutil.copy(interpreter_binary(), app / "Contents" / "MacOS" / "Test")
    with open(app / "Contents" / "Info.plist", "wb") as f:
        plistlib.dump({"LSMinimumSystemVersion": declared}, f)
    return app


def test_check_bundle_fails_when_info_plist_promises_too_old_a_macos(tmp_path, capsys):
    needed = macho_minos.minos(interpreter_binary())
    assert macho_minos.check_bundle(make_bundle(tmp_path / "ok", macho_minos.as_text(needed))) == 0
    if needed > (10, 9, 0):
        assert macho_minos.check_bundle(make_bundle(tmp_path / "low", "10.9")) == 1
        assert "need newer" in capsys.readouterr().out
