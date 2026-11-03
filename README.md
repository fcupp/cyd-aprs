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
