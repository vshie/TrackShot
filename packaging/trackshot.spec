# PyInstaller spec for TrackShot: onedir, windowed. macOS arm64 → TrackShot.app; Linux and Windows →
# a TrackShot/ folder with the TrackShot (or TrackShot.exe) launcher.
#
#     .venv/bin/pyinstaller --noconfirm --clean packaging/trackshot.spec
#
# (scripts/build_macos.sh runs the tests, this spec, ad-hoc signing and the bundle's self-test;
# .github/workflows/build.yml builds all three platforms.)
#
# What needs care, and why:
# - depthai is a single extension module (depthai.cpython-3XX-darwin.so) linking
#   palace.dylibs/libdepthai-core.dylib (which embeds the device firmware, RTAB-Map and Basalt)
#   plus libdynamic_calibration and libusb via @loader_path. PyInstaller's Mach-O dependency
#   scan follows those links; the explicit entries below make sure they are bundled even if a
#   future wheel loads one with dlopen. collect_all('depthai') finds nothing in 3.12 (a single
#   module is not a package) but is kept for later wheels, filtered to the running interpreter's
#   extension suffix: the wheel carries a .so for every Python version (~50 MB we don't want).
#   On Linux and Windows the native libraries sit in a sibling folder the wheel repair tool made
#   (auditwheel/delvewheel: depthai.libs or similar) or next to the module; those are bundled
#   under the same relative folder so the extension's RPATH / DLL search still finds them.
# - pymavlink imports its dialect modules by name at runtime (mavutil.set_dialect), which static
#   analysis can't see: the dialects TrackShot uses are listed as hidden imports.
# - PySide6 is 1.2 GB installed; only QtCore/QtGui/QtWidgets (+ what pyqtgraph touches) are used,
#   so the big optional modules are excluded.
# - The fake autopilot (trackshot.testing) and test-only packages stay out of the app.
# - LSMinimumSystemVersion is the highest deployment target (minos) among the bundled binaries
#   (macho_minos.py), not a hopeful constant: a Homebrew Python is built for the macOS it was
#   compiled on, and dyld refuses it on anything older. build_macos.sh re-checks the bundle.
#   The .app bundle, minos and arm64 target apply on macOS only.

import importlib.machinery
import sys
from pathlib import Path

from PyInstaller.utils.hooks import collect_all, copy_metadata

ROOT = Path(SPECPATH).parent          # noqa: F821 - SPECPATH is defined by PyInstaller
SRC = ROOT / "src"
MACOS = sys.platform == "darwin"
WINDOWS = sys.platform == "win32"
ICON = ROOT / "packaging" / ("TrackShot.icns" if MACOS else "TrackShot.ico")
sys.path.insert(0, str(SRC))
sys.path.insert(0, SPECPATH)          # noqa: F821
from trackshot.__main__ import APP_NAME, app_version  # noqa: E402

VERSION = app_version()

# --- depthai -------------------------------------------------------------------------------------
dai_datas, dai_binaries, dai_hidden = collect_all("depthai")
ext_suffix = importlib.machinery.EXTENSION_SUFFIXES[0]       # e.g. ".cpython-314-darwin.so"


def wanted(entry) -> bool:
    name = Path(entry[0]).name
    return not (name.startswith("depthai.cpython-") and not name.endswith(ext_suffix)) \
        and not name.endswith(".pyi")


dai_datas = [d for d in dai_datas if wanted(d)]
dai_binaries = [b for b in dai_binaries if wanted(b)]
import depthai  # noqa: E402

site = Path(depthai.__file__).parent
NATIVE = ("*.dylib", "*.so", "*.so.*", "*.dll", "*.pyd")
for libdir in [site / "palace.dylibs", *sorted(site.glob("depthai*.libs"))]:
    if libdir.is_dir():
        for pattern in NATIVE:
            dai_binaries += [(str(lib), libdir.name) for lib in sorted(libdir.glob(pattern))]
if WINDOWS:
    # depthai is a single module in site-packages, so DLLs the wheel puts beside it land in
    # site-packages itself: take only depthai's own (by name), not every package's
    DAI_DLL_PREFIXES = ("depthai", "libdepthai", "dynamic", "libdynamic", "libusb")
    dai_binaries += [(str(lib), ".") for lib in sorted(site.glob("*.dll"))
                     if lib.name.lower().startswith(DAI_DLL_PREFIXES)]
dai_binaries = list(dict.fromkeys(dai_binaries))
print(f"depthai native libraries bundled: {len(dai_binaries)}")

# --- Python-level ---------------------------------------------------------------------------------
hiddenimports = dai_hidden + [
    "pymavlink.dialects.v20.ardupilotmega",
    "pymavlink.dialects.v20.common",
    "pymavlink.dialects.v20.minimal",
    "pymavlink.dialects.v10.ardupilotmega",   # mavutil's import-time default dialect
    "trackshot.vio.oak_pipeline",              # imported lazily by the engine
    "trackshot.vio.sim",
    "trackshot.capture.meta",
]

datas = dai_datas + [(str(SRC / "trackshot" / "resources" / name), "trackshot/resources")
                     for name in ("icon.png", "widget.html")]
try:
    datas += copy_metadata("trackshot")       # so the app can report its version
except Exception:  # noqa: BLE001 - not installed as a distribution: the version shows "dev"
    pass

excludes = [
    # Tests and dev-only
    "trackshot.testing", "tests", "pytest", "pytestqt", "_pytest", "websockets", "PyInstaller",
    # Big optional Qt modules TrackShot doesn't use
    "PySide6.QtWebEngineCore", "PySide6.QtWebEngineWidgets", "PySide6.QtWebEngineQuick", "PySide6.QtWebChannel",
    "PySide6.QtWebSockets", "PySide6.QtQml", "PySide6.QtQuick", "PySide6.QtQuick3D", "PySide6.QtQuickWidgets",
    "PySide6.QtQuickControls2", "PySide6.Qt3DCore", "PySide6.Qt3DRender", "PySide6.Qt3DInput",
    "PySide6.Qt3DLogic", "PySide6.Qt3DExtras", "PySide6.Qt3DAnimation", "PySide6.QtMultimedia",
    "PySide6.QtMultimediaWidgets", "PySide6.QtCharts", "PySide6.QtDataVisualization", "PySide6.QtGraphs",
    "PySide6.QtPdf", "PySide6.QtPdfWidgets", "PySide6.QtSql", "PySide6.QtTest", "PySide6.QtBluetooth",
    "PySide6.QtNfc", "PySide6.QtSensors", "PySide6.QtSerialPort", "PySide6.QtSerialBus",
    "PySide6.QtPositioning", "PySide6.QtLocation", "PySide6.QtDesigner", "PySide6.QtHelp",
    "PySide6.QtUiTools", "PySide6.QtRemoteObjects", "PySide6.QtScxml", "PySide6.QtStateMachine",
    "PySide6.QtTextToSpeech", "PySide6.QtSpatialAudio", "PySide6.QtHttpServer", "PySide6.QtNetworkAuth",
    "PySide6.QtVirtualKeyboard", "PySide6.QtAxContainer", "PySide6.QtCanvasPainter",
    # Other GUI toolkits and heavy optional imports pyqtgraph/PIL probe for
    "PyQt5", "PyQt6", "tkinter", "matplotlib", "scipy", "IPython", "jupyter_client", "cupy", "numba",
    "h5py", "pandas", "OpenGL", "pyqtgraph.opengl", "pyqtgraph.examples", "pyqtgraph.jupyter",
    "depthai_cli",
    "lxml",            # pymavlink imports it only to replay DataFlash logs
]

# Qt plugins PyInstaller's PySide6 hook collects by default that drag in frameworks TrackShot never
# uses: the virtual keyboard (→ QtQuick/QtQml) and the PDF image format (→ QtPdf). ~25 MB.
UNUSED_QT = ("QtVirtualKeyboard", "QtQuick", "QtQml", "QtPdf", "libqtvirtualkeyboardplugin", "libqpdf",
             "Qt6VirtualKeyboard", "Qt6Quick", "Qt6Qml", "Qt6Pdf", "qtvirtualkeyboardplugin", "qpdf.dll")


def keep_qt(entry) -> bool:
    return not any(part in entry[0] for part in UNUSED_QT)

a = Analysis(  # noqa: F821
    [str(SRC / "trackshot" / "__main__.py")],
    pathex=[str(SRC)],
    binaries=dai_binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    excludes=excludes,
    noarchive=False,
)
a.binaries = [b for b in a.binaries if keep_qt(b)]
a.datas = [d for d in a.datas if keep_qt(d)]
pyz = PYZ(a.pure)  # noqa: F821
if MACOS:
    from macho_minos import required_macos  # noqa: E402
    MIN_MACOS = required_macos(Path(entry[1]) for entry in a.binaries)
    print(f"TrackShot.app needs macOS {MIN_MACOS} or later (the bundled binaries' highest minos)")

exe = EXE(  # noqa: F821
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name=APP_NAME,
    console=False,
    argv_emulation=False,
    target_arch="arm64" if MACOS else None,
    upx=False,
    codesign_identity=None,       # build_macos.sh signs the finished bundle ad hoc
    entitlements_file=None,
    icon=str(ICON) if (MACOS or WINDOWS) else None,
)

coll = COLLECT(  # noqa: F821
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    name=APP_NAME,
)

if MACOS:
    app = BUNDLE(  # noqa: F821
        coll,
        name=f"{APP_NAME}.app",
        icon=str(ICON),
        bundle_identifier="com.bluerobotics.trackshot",
        version=VERSION,
        info_plist={
            "CFBundleName": APP_NAME,
            "CFBundleDisplayName": APP_NAME,
            "CFBundleShortVersionString": VERSION,
            "CFBundleVersion": VERSION,
            "LSMinimumSystemVersion": MIN_MACOS,
            "LSApplicationCategoryType": "public.app-category.utilities",
            "NSHighResolutionCapable": True,
            "NSRequiresAquaSystemAppearance": False,     # follow the system's dark mode
            # macOS 15+ blocks LAN traffic for apps without this (and the user's consent)
            "NSLocalNetworkUsageDescription": (
                "TrackShot connects to the ROV's OAK-D camera and to the autopilot (BlueOS) over the "
                "tether's local network to run visual odometry and send position updates."),
            "NSHumanReadableCopyright": "TrackShot",
        },
    )
