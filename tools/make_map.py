#!/usr/bin/env python3
"""Make map.raw (320x240 RGB565, little-endian) for the CYD APRS map.
Copy the result to the root of a FAT32 microSD card as /map.raw.

  pip install pillow requests
  python tools/make_map.py LAT LON RADIUS_KM

LAT/LON/RADIUS must match what you enter in the device's Wi-Fi setup page.
Tiles default to CARTO dark (© OpenStreetMap contributors © CARTO). Run it once, not in a loop.
"""
import argparse, io, math, struct, sys, time
import requests
from PIL import Image

W, H = 320, 240

def merc_x(lon): return (lon + 180.0) / 360.0
def merc_y(lat):
    s = math.sin(math.radians(lat))
    return 0.5 - math.log((1 + s) / (1 - s)) / (4 * math.pi)

ap = argparse.ArgumentParser()
ap.add_argument("lat", type=float); ap.add_argument("lon", type=float); ap.add_argument("radius_km", type=float)
ap.add_argument("--tiles", default="https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png")
ap.add_argument("--out", default="map.raw"); ap.add_argument("--preview", default="map_preview.png")
a = ap.parse_args()

world_w = 2 * a.radius_km * 1000 / (40075016.686 * math.cos(math.radians(a.lat)))
world_h = world_w * H / W
x0 = merc_x(a.lon) - world_w / 2
y0 = merc_y(a.lat) - world_h / 2

z = max(0, min(17, math.ceil(math.log2(2 * W / (256 * world_w)))))
n = 2 ** z
tx0, tx1 = int(x0 * n), int((x0 + world_w) * n)
ty0, ty1 = int(y0 * n), int((y0 + world_h) * n)
count = (tx1 - tx0 + 1) * (ty1 - ty0 + 1)
if count > 36: sys.exit(f"Refusing to fetch {count} tiles; use a smaller radius.")
print(f"zoom {z}, {count} tiles")

canvas = Image.new("RGB", ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256))
hdr = {"User-Agent": "cyd-aprs-map/0.1 (personal hobby project)"}
for tx in range(tx0, tx1 + 1):
    for ty in range(ty0, ty1 + 1):
        r = requests.get(a.tiles.format(z=z, x=tx, y=ty), headers=hdr, timeout=20)
        r.raise_for_status()
        canvas.paste(Image.open(io.BytesIO(r.content)).convert("RGB"), ((tx - tx0) * 256, (ty - ty0) * 256))
        time.sleep(0.2)

left, top = (x0 * n - tx0) * 256, (y0 * n - ty0) * 256
img = canvas.crop((round(left), round(top), round(left + world_w * n * 256), round(top + world_h * n * 256)))
img = img.resize((W, H), Image.LANCZOS)
img.save(a.preview)

out = bytearray()
for r, g, b in img.getdata():
    out += struct.pack("<H", ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))
open(a.out, "wb").write(out)
print(f"wrote {a.out} ({len(out)} bytes) and {a.preview}")
