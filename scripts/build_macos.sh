#!/usr/bin/env bash
# Builds dist/TrackShot.app (macOS arm64): tests, PyInstaller, ad-hoc signing, then the bundle's
# own self-test. Run from anywhere:
#
#     scripts/build_macos.sh            # full build
#     SKIP_TESTS=1 scripts/build_macos.sh
#     PYTHON=/path/to/python scripts/build_macos.sh   # e.g. CI: an interpreter other than .venv's
#     DIST=dist-next scripts/build_macos.sh           # build beside a copy that is running
#
# The app is signed ad hoc ("-"), which is enough to launch it on this Mac. On another Mac,
# Gatekeeper will refuse it until it is opened with right-click → Open (or signed and notarised
# with a Developer ID).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PY="${PYTHON:-$ROOT/.venv/bin/python}"
DIST="${DIST:-dist}"
[[ "$DIST" = /* ]] || DIST="$ROOT/$DIST"
APP="$DIST/TrackShot.app"

if [[ "$(uname -s)" != "Darwin" || "$(uname -m)" != "arm64" ]]; then
    echo "This script builds the macOS arm64 app; this machine is $(uname -s) $(uname -m)." >&2
    exit 1
fi
if [[ ! -x "$PY" ]]; then
    echo "No Python at $PY: python3 -m venv .venv && .venv/bin/pip install -e '.[dev]' (or set PYTHON)" >&2
    exit 1
fi

# Never replace a bundle that is running. A PyInstaller app imports modules lazily from its own
# archive; swap the archive underneath it and every module it hasn't loaded yet fails to import
# (seen as "unknown encoding: idna" the first time the running app talked to the autopilot).
if pgrep -f "$APP/Contents/MacOS/TrackShot" >/dev/null; then
    echo "TrackShot is running from $APP. Quit it first, then build again." >&2
    exit 1
fi

if [[ -z "${SKIP_TESTS:-}" ]]; then
    echo "==> Tests"
    QT_QPA_PLATFORM=offscreen "$PY" -m pytest -q
fi

if [[ ! -f packaging/TrackShot.icns || ! -f src/trackshot/resources/icon.png ]]; then
    echo "==> Icon"
    "$PY" scripts/make_icon.py
fi

echo "==> PyInstaller"
"$PY" -m PyInstaller --noconfirm --clean --log-level WARN \
    --distpath "$DIST" --workpath "$ROOT/build" packaging/trackshot.spec

echo "==> Minimum macOS"
# Fails if any bundled binary needs a newer macOS than Info.plist's LSMinimumSystemVersion
"$PY" packaging/macho_minos.py "$APP"

echo "==> Ad-hoc signing"
# --deep signs every nested library and framework; the bundle won't launch with stale signatures
codesign --force --deep --sign - "$APP"
codesign --verify --deep --strict "$APP"

echo "==> Bundle self-test"
"$APP/Contents/MacOS/TrackShot" --self-test

echo
echo "Built: $APP (macOS $(/usr/libexec/PlistBuddy -c 'Print :LSMinimumSystemVersion' "$APP/Contents/Info.plist") or later)"
du -sh "$APP" | awk '{print "Size:  " $1}'
