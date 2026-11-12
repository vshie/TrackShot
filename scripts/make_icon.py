"""Draws TrackShot's app icon and writes the PNG the app uses plus the macOS .icns.

The icon is a camera lens on a deep-water tile with a survey track sweeping around it: the app
is "the camera's eye, turned into a track". It is drawn in code so it can be regenerated and
tweaked without design tools.

    .venv/bin/python scripts/make_icon.py

Writes src/trackshot/resources/icon.png (1024 px) and packaging/TrackShot.icns (via macOS's
`sips` and `iconutil`; the .icns step is skipped elsewhere).
"""

from __future__ import annotations

import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageChops, ImageDraw, ImageFilter

ROOT = Path(__file__).resolve().parents[1]
PNG_PATH = ROOT / "src" / "trackshot" / "resources" / "icon.png"
ICNS_PATH = ROOT / "packaging" / "TrackShot.icns"

SIZE = 1024
SS = 3                      # supersampling: draw at 3x, then downscale for smooth edges
# Apple's icon grid: an 824 px tile centred on the 1024 canvas, ~185 px corner radius
TILE = 824
RADIUS = 185

DEEP = (6, 30, 58)
SHALLOW = (12, 104, 140)
TRACK = (255, 196, 61)      # amber: reads on blue in light and dark docks
GLASS_RIM = (226, 236, 244)


def s(v: float) -> int:
    return int(round(v * SS))


def vertical_gradient(size: tuple[int, int], top: tuple, bottom: tuple) -> Image.Image:
    w, h = size
    column = Image.new("RGB", (1, h))
    for y in range(h):
        t = y / max(1, h - 1)
        column.putpixel((0, y), tuple(int(a + (b - a) * t) for a, b in zip(top, bottom)))
    return column.resize((w, h))


def radial_alpha(size: int, inner: float, outer: float) -> Image.Image:
    """An alpha mask fading from opaque at radius `inner` to clear at `outer` (fractions of size/2)."""
    mask = Image.new("L", (size, size), 0)
    draw = ImageDraw.Draw(mask)
    steps = 64
    for i in range(steps, -1, -1):
        frac = inner + (outer - inner) * i / steps
        r = frac * size / 2
        level = int(255 * (1 - i / steps))
        c = size / 2
        draw.ellipse((c - r, c - r, c + r, c + r), fill=level)
    return mask


def draw_icon() -> Image.Image:
    big = SIZE * SS
    canvas = Image.new("RGBA", (big, big), (0, 0, 0, 0))
    x0 = (SIZE - TILE) / 2
    tile_box = (s(x0), s(x0), s(x0 + TILE), s(x0 + TILE))

    # Soft drop shadow under the tile, as macOS icons have
    shadow = Image.new("L", (big, big), 0)
    ImageDraw.Draw(shadow).rounded_rectangle(
        (tile_box[0], tile_box[1] + s(14), tile_box[2], tile_box[3] + s(14)), radius=s(RADIUS), fill=110)
    shadow = shadow.filter(ImageFilter.GaussianBlur(s(18)))
    canvas.paste((0, 0, 0, 255), (0, 0), shadow)

    # Water: a vertical gradient with a lighter patch where the light comes from
    tile_mask = Image.new("L", (big, big), 0)
    ImageDraw.Draw(tile_mask).rounded_rectangle(tile_box, radius=s(RADIUS), fill=255)
    water = vertical_gradient((big, big), SHALLOW, DEEP).convert("RGBA")
    glow = Image.new("RGBA", (big, big), (90, 200, 220, 0))
    glow.putalpha(radial_alpha(big, 0.0, 0.9).point(lambda v: int(v * 0.35)))
    glow = glow.transform((big, big), Image.Transform.AFFINE, (1, 0, s(110), 0, 1, s(170)))
    water = Image.alpha_composite(water, glow)
    canvas.paste(water, (0, 0), tile_mask)

    draw = ImageDraw.Draw(canvas)
    cx, cy = SIZE / 2, SIZE / 2 + 10
    lens_r = 205

    # Survey track: a lawnmower path behind the lens, dashed, ending in a heading arrow
    track = []
    lane_y = [cy - 250, cy - 125, cy, cy + 125, cy + 250]
    left, right = SIZE / 2 - 285, SIZE / 2 + 285
    for i, y in enumerate(lane_y):
        a, b = (left, right) if i % 2 == 0 else (right, left)
        track.append(((a, y), (b, y)))
    dash, gap = 30, 42          # round caps add a width to each dash, so the gap must exceed it
    width = 22
    points: list[tuple[float, float]] = []
    for i, ((ax, ay), (bx, by)) in enumerate(track):
        points += [(ax, ay), (bx, by)]
        if i + 1 < len(track):
            # Semicircular turn to the next lane
            r = (track[i + 1][0][1] - by) / 2
            centre = (bx, by + r)
            sign = 1 if bx > ax else -1
            for k in range(1, 24):
                th = -math.pi / 2 + math.pi * k / 24
                points.append((centre[0] + sign * r * math.cos(th), centre[1] + r * math.sin(th)))
    def dashes(pts):
        """Splits the polyline into dashes, each a list of points (so a dash bends round a turn)."""
        current, carry, on = [pts[0]], 0.0, True
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            seg = math.hypot(bx - ax, by - ay)
            pos = 0.0
            while pos < seg:
                step = min((dash if on else gap) - carry, seg - pos)
                pos += step
                carry += step
                point = (ax + (bx - ax) * pos / seg, ay + (by - ay) * pos / seg)
                if on:
                    current.append(point)
                if carry >= (dash if on else gap) - 1e-6:
                    if on:
                        yield current
                    current, carry, on = [point], 0.0, not on
        if on and len(current) > 1:
            yield current
    track_layer = Image.new("RGBA", (big, big), (0, 0, 0, 0))
    tdraw = ImageDraw.Draw(track_layer)
    r = width / 2
    for piece in dashes(points):
        # Stamp discs along the dash: Pillow's thick-line joints bulge on tight curves
        for (ax, ay), (bx, by) in zip(piece, piece[1:]):
            n = max(1, int(math.hypot(bx - ax, by - ay) * 2))
            for k in range(n + 1):
                px, py = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
                tdraw.ellipse((s(px - r), s(py - r), s(px + r), s(py + r)), fill=TRACK + (235,))
    # Heading arrow at the end of the last lane
    ex, ey = points[-1]
    direction = 1 if points[-1][0] > points[-2][0] else -1
    tdraw.polygon([(s(ex + direction * 46), s(ey)), (s(ex - direction * 6), s(ey - 40)),
                   (s(ex - direction * 6), s(ey + 40))], fill=TRACK + (255,))
    # Fade the track where it passes behind the lens, so the lens reads as in front
    clear = Image.new("L", (big, big), 255)
    ImageDraw.Draw(clear).ellipse((s(cx - lens_r - 30), s(cy - lens_r - 30),
                                   s(cx + lens_r + 30), s(cy + lens_r + 30)), fill=0)
    track_layer.putalpha(ImageChops.multiply(track_layer.getchannel("A"), clear))
    canvas = Image.alpha_composite(canvas, Image.composite(track_layer, Image.new("RGBA", (big, big)),
                                                           tile_mask))
    draw = ImageDraw.Draw(canvas)

    # Lens: rim, dark barrel rings, glass with a teal iris and a highlight
    def circle(r, **kw):
        draw.ellipse((s(cx - r), s(cy - r), s(cx + r), s(cy + r)), **kw)
    circle(lens_r + 8, fill=(3, 14, 28, 120))
    circle(lens_r, fill=GLASS_RIM + (255,))
    circle(lens_r - 22, fill=(20, 28, 38, 255))
    circle(lens_r - 50, fill=(36, 48, 62, 255))
    circle(lens_r - 62, fill=(8, 12, 20, 255))
    iris = Image.new("RGBA", (big, big), (0, 0, 0, 0))
    ImageDraw.Draw(iris).ellipse((s(cx - 105), s(cy - 105), s(cx + 105), s(cy + 105)), fill=(20, 150, 170, 255))
    iris_mask = radial_alpha(big, 0.0, 0.25).transform(
        (big, big), Image.Transform.AFFINE, (1, 0, s(SIZE / 2 - cx), 0, 1, s(SIZE / 2 - cy)))
    iris.putalpha(ImageChops.multiply(iris.getchannel("A"), iris_mask))
    canvas = Image.alpha_composite(canvas, iris)
    draw = ImageDraw.Draw(canvas)
    circle(46, fill=(4, 8, 14, 255))
    # Specular highlights
    draw.ellipse((s(cx - 98), s(cy - 112), s(cx - 30), s(cy - 64)), fill=(255, 255, 255, 170))
    draw.ellipse((s(cx + 40), s(cy + 48), s(cx + 66), s(cy + 70)), fill=(255, 255, 255, 110))
    # A short arc on the rim: the camera's "tracking" mark
    draw.arc((s(cx - lens_r + 10), s(cy - lens_r + 10), s(cx + lens_r - 10), s(cy + lens_r - 10)),
             start=200, end=250, fill=TRACK + (255,), width=s(10))

    return canvas.resize((SIZE, SIZE), Image.Resampling.LANCZOS)


def write_icns(png: Path, icns: Path) -> None:
    """iconutil wants an .iconset of the standard sizes; sips resamples the master PNG."""
    with tempfile.TemporaryDirectory() as tmp:
        iconset = Path(tmp) / "TrackShot.iconset"
        iconset.mkdir()
        for base in (16, 32, 128, 256, 512):
            for scale in (1, 2):
                px = base * scale
                name = f"icon_{base}x{base}{'@2x' if scale == 2 else ''}.png"
                subprocess.run(["sips", "-z", str(px), str(px), str(png), "--out", str(iconset / name)],
                               check=True, capture_output=True)
        icns.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["iconutil", "-c", "icns", str(iconset), "-o", str(icns)], check=True)


def main() -> int:
    PNG_PATH.parent.mkdir(parents=True, exist_ok=True)
    draw_icon().save(PNG_PATH)
    print(f"wrote {PNG_PATH}")
    if sys.platform == "darwin" and shutil.which("iconutil") and shutil.which("sips"):
        write_icns(PNG_PATH, ICNS_PATH)
        print(f"wrote {ICNS_PATH}")
    else:
        print("not macOS: skipped the .icns")
    return 0


if __name__ == "__main__":
    sys.exit(main())
