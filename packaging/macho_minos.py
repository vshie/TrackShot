"""The oldest macOS each Mach-O binary can load on (its `minos`), for the app bundle's
LSMinimumSystemVersion.

Every binary records the deployment target it was built for (LC_BUILD_VERSION, or the older
LC_VERSION_MIN_MACOSX), and dyld refuses one built for a newer macOS than the one running. A
Homebrew Python, for instance, is built for the macOS it was compiled on, so a bundle made from it
needs that macOS no matter what Info.plist claims. The spec sets LSMinimumSystemVersion to the
highest minos among the binaries it bundles, and build_macos.sh checks the finished bundle:

    .venv/bin/python packaging/macho_minos.py dist/TrackShot.app     # exit 1 if Info.plist is too low
"""

from __future__ import annotations

import plistlib
import sys
from collections import Counter
from pathlib import Path
from typing import Iterable, Optional

from macholib import mach_o
from macholib.MachO import MachO

Version = tuple[int, int, int]


def _decode(packed: int) -> Version:
    # xxxx.yy.zz in nibbles: 0x1B0000 is 27.0.0
    return packed >> 16, (packed >> 8) & 0xFF, packed & 0xFF


def minos(path: Path) -> Optional[Version]:
    """The binary's minimum macOS version (the highest across its architectures), or None if it
    isn't a Mach-O file or records none."""
    try:
        macho = MachO(str(path))
    except Exception:   # not a Mach-O file (or unreadable): nothing to load
        return None
    found: list[Version] = []
    for header in macho.headers:
        for load_command, command, _data in header.commands:
            if load_command.cmd == mach_o.LC_BUILD_VERSION:
                found.append(_decode(command.minos))
            elif load_command.cmd == mach_o.LC_VERSION_MIN_MACOSX:
                found.append(_decode(command.version))
    return max(found) if found else None


def scan(paths: Iterable[Path]) -> dict[Path, Version]:
    """minos of every Mach-O file among `paths` (others are skipped)."""
    result = {}
    for path in paths:
        path = Path(path)
        if path.is_file() and not path.is_symlink():
            version = minos(path)
            if version is not None:
                result[path] = version
    return result


def as_text(version: Version) -> str:
    major, minor, patch = version
    return f"{major}.{minor}" + (f".{patch}" if patch else "")


def parse(text: str) -> Version:
    parts = [int(p) for p in text.split(".")] + [0, 0]
    return parts[0], parts[1], parts[2]


def required_macos(paths: Iterable[Path], floor: str = "11.0") -> str:
    """LSMinimumSystemVersion for a bundle of these binaries: the highest minos, at least `floor`
    (the first macOS on Apple silicon)."""
    versions = list(scan(paths).values())
    return as_text(max(versions + [parse(floor)]))


def check_bundle(app: Path) -> int:
    """Prints the bundle's minos histogram; 1 if a binary needs a newer macOS than Info.plist says."""
    with open(app / "Contents" / "Info.plist", "rb") as f:
        declared = parse(plistlib.load(f)["LSMinimumSystemVersion"])
    found = scan(p for p in (app / "Contents").rglob("*"))
    for version, count in sorted(Counter(found.values()).items()):
        print(f"  minos {as_text(version):>6}: {count} binaries")
    too_new = sorted((v, p) for p, v in found.items() if v > declared)
    if too_new:
        print(f"LSMinimumSystemVersion is {as_text(declared)}, but {len(too_new)} binaries need newer, e.g.:")
        for version, path in too_new[-5:]:
            print(f"  {as_text(version)}  {path.relative_to(app)}")
        return 1
    print(f"LSMinimumSystemVersion {as_text(declared)} covers all {len(found)} binaries")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(f"usage: {sys.argv[0]} path/to/App.app")
    sys.exit(check_bundle(Path(sys.argv[1])))
