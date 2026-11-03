# CYD APRS Map
ESP32 CYD (ESP32-2432S028) showing nearby APRS stations from APRS-IS (read-only, `pass -1`).

## Build & web-flash
1. Push this folder to a GitHub repo (branch `main`), enable **Settings → Pages → Source: GitHub Actions**.
2. The workflow builds both board variants and publishes the flasher page. Open it in Chrome/Edge and click the button matching your board.
3. Join **CYD-APRS-Setup** (192.168.4.1) and enter Wi-Fi, callsign, home lat/lon, radius km.

## Map background (optional)
`python tools/make_map.py LAT LON RADIUS_KM` → copy `map.raw` to a FAT32 microSD as `/map.raw`. Without it you get a grid.

## Use
BOOT button: toggle map/list. Hold BOOT while powering up to wipe Wi-Fi/config.
Colours inverted? Add `-DCYD_FLIP_INVERT=1` to build_flags. Wrong panel? Try the other env.
Local build: `pio run -e cyd -t upload`.

How it works

LIST button: a small button in the top-right corner of the map opens the list. 
The touch area is bigger than the drawn button, so you don’t have to hit it exactly.

List view: stations are shown newest first, 8 per page, with callsign, distance, and minutes since last heard. Each row is about 24 pixels tall, much easier to hit than a dot. PREV and NEXT page through the list, and MAP goes back.

Detail screen: tapping a row shows the callsign in large type, position, distance and bearing from home, time since last heard, and the APRS symbol. BACK returns to the list, and SHOW ON MAP jumps to the map, zoomed 2× and centered on that station with its info panel open.

List stays steady: list and detail screens only redraw when you tap, or every 30 seconds, so they won’t flicker while you’re aiming.

Unchanged: tapping stations on the map still works, the top bar still resets zoom, and the BOOT button still toggles between map and list.
