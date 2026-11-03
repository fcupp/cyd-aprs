// CYD APRS map: APRS-IS (read-only) -> 320x240 map with station dots.
// Touch: tap a station = details; tap empty map = zoom in 2x (max 4x) centred there;
//        tap the top status bar = reset zoom. Tap list view = back to map.
// BOOT button: short press = toggle map/list, hold >1.5 s = recalibrate touch,
//              hold at power-up = reset Wi-Fi/config.
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>
#include <algorithm>
#include "aprs.h"

#define PIN_BL 21
#define PIN_BOOT 0
#define SD_CS 5
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define T_CLK 25
#define T_MOSI 32
#define T_MISO 39
#define T_CS 33
#define T_IRQ 36
#define SCR_W 320
#define SCR_H 240
#define MAX_ST 48
#define EXPIRE_MS (60UL * 60UL * 1000UL)

TFT_eSPI tft;
SPIClass sdSPI(HSPI);
Preferences prefs;
WiFiClient client;

char cCall[12] = "N0CALL", cServer[40] = "rotate.aprs2.net", cPort[8] = "14580";
char cLat[12] = "0.0", cLon[12] = "0.0", cRad[8] = "50";

Station st[MAX_ST];
bool haveMap = false, dirty = true, listView = false, aprsOk = false;
double homeLat, homeLon, radiusKm, wx0, wy0, worldW, worldH;   // full extent
double vx0, vy0, vW, vH; int zf = 1;                           // current view
int selIdx = -1;
char lineBuf[512]; size_t lineLen = 0;
uint32_t lastKeep = 0, lastTry = 0, lastDraw = 0, pktCount = 0;

struct Cal { bool valid = false, swap = false; int a0 = 0, a1 = 0, b0 = 0, b1 = 0; } cal;

// ---- projection (must match tools/make_map.py) ----
static double mercX(double lon) { return (lon + 180.0) / 360.0; }
static double mercY(double lat) { double s = sin(lat * DEG_TO_RAD); return 0.5 - log((1 + s) / (1 - s)) / (4 * PI); }
void setView(double cx, double cy, int z) {
  zf = z; vW = worldW / z; vH = worldH / z;
  cx = constrain(cx, wx0 + vW / 2, wx0 + worldW - vW / 2);
  cy = constrain(cy, wy0 + vH / 2, wy0 + worldH - vH / 2);
  vx0 = cx - vW / 2; vy0 = cy - vH / 2;
}
void setupProjection() {
  worldW = 2.0 * radiusKm * 1000.0 / (40075016.686 * cos(homeLat * DEG_TO_RAD));
  worldH = worldW * SCR_H / SCR_W;
  wx0 = mercX(homeLon) - worldW / 2; wy0 = mercY(homeLat) - worldH / 2;
  setView(wx0 + worldW / 2, wy0 + worldH / 2, 1);
}
bool project(double lat, double lon, int& x, int& y) {
  x = (int)((mercX(lon) - vx0) / vW * SCR_W);
  y = (int)((mercY(lat) - vy0) / vH * SCR_H);
  return x >= 0 && x < SCR_W && y >= 0 && y < SCR_H;
}
double distKm(double la1, double lo1, double la2, double lo2) {
  double dl = (la2 - la1) * DEG_TO_RAD, dn = (lo2 - lo1) * DEG_TO_RAD;
  double a = sin(dl / 2) * sin(dl / 2) + cos(la1 * DEG_TO_RAD) * cos(la2 * DEG_TO_RAD) * sin(dn / 2) * sin(dn / 2);
  return 12742.0 * asin(sqrt(a));
}
double bearingDeg(double la1, double lo1, double la2, double lo2) {
  double p1 = la1 * DEG_TO_RAD, p2 = la2 * DEG_TO_RAD, dl = (lo2 - lo1) * DEG_TO_RAD;
  double y = sin(dl) * cos(p2), x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
  double b = atan2(y, x) * RAD_TO_DEG; return b < 0 ? b + 360 : b;
}

// ---- config ----
void loadCfg() {
  prefs.begin("aprs", true);
  strlcpy(cCall, prefs.getString("call", cCall).c_str(), sizeof(cCall));
  strlcpy(cServer, prefs.getString("server", cServer).c_str(), sizeof(cServer));
  strlcpy(cPort, prefs.getString("port", cPort).c_str(), sizeof(cPort));
  strlcpy(cLat, prefs.getString("lat", cLat).c_str(), sizeof(cLat));
  strlcpy(cLon, prefs.getString("lon", cLon).c_str(), sizeof(cLon));
  strlcpy(cRad, prefs.getString("rad", cRad).c_str(), sizeof(cRad));
  cal.valid = prefs.getBool("cv", false); cal.swap = prefs.getBool("cs", false);
  cal.a0 = prefs.getInt("ca0", 0); cal.a1 = prefs.getInt("ca1", 0);
  cal.b0 = prefs.getInt("cb0", 0); cal.b1 = prefs.getInt("cb1", 0);
  prefs.end();
}
void saveCfg() {
  prefs.begin("aprs", false);
  prefs.putString("call", cCall); prefs.putString("server", cServer); prefs.putString("port", cPort);
  prefs.putString("lat", cLat); prefs.putString("lon", cLon); prefs.putString("rad", cRad);
  prefs.end();
}
void saveCal() {
  prefs.begin("aprs", false);
  prefs.putBool("cv", cal.valid); prefs.putBool("cs", cal.swap);
  prefs.putInt("ca0", cal.a0); prefs.putInt("ca1", cal.a1);
  prefs.putInt("cb0", cal.b0); prefs.putInt("cb1", cal.b1);
  prefs.end();
}
void msg(const char* a, const char* b = "") {
  tft.fillScreen(TFT_BLACK); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(a, 8, 8, 2); tft.drawString(b, 8, 30, 2);
}
void redraw() { dirty = true; lastDraw = 0; }

// ---- touch (XPT2046, bit-banged: its pins are separate from the TFT/SD buses) ----
void touchInit() {
  pinMode(T_CLK, OUTPUT); pinMode(T_MOSI, OUTPUT); pinMode(T_CS, OUTPUT);
  pinMode(T_MISO, INPUT); pinMode(T_IRQ, INPUT);
  digitalWrite(T_CS, HIGH); digitalWrite(T_CLK, LOW);
}
static uint16_t xptRead(uint8_t cmd) {
  digitalWrite(T_CS, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(T_MOSI, (cmd >> i) & 1);
    digitalWrite(T_CLK, HIGH); delayMicroseconds(1); digitalWrite(T_CLK, LOW); delayMicroseconds(1);
  }
  uint16_t v = 0;
  for (int i = 0; i < 16; i++) {
    digitalWrite(T_CLK, HIGH); delayMicroseconds(1); digitalWrite(T_CLK, LOW); delayMicroseconds(1);
    v = (v << 1) | digitalRead(T_MISO);
  }
  digitalWrite(T_CS, HIGH);
  return (v >> 3) & 0x0FFF;
}
bool readRaw(int& rx, int& ry) {
  if (digitalRead(T_IRQ)) return false;
  long sx = 0, sy = 0;
  for (int i = 0; i < 4; i++) { sx += xptRead(0xD0); sy += xptRead(0x90); }
  rx = sx / 4; ry = sy / 4;
  return rx > 80 && rx < 4015 && ry > 80 && ry < 4015;
}
bool touchScreen(int& x, int& y) {
  int rx, ry;
  if (!cal.valid || !readRaw(rx, ry)) return false;
  int a = cal.swap ? ry : rx, b = cal.swap ? rx : ry;
  x = 20 + (long)(a - cal.a0) * 280 / (cal.a1 - cal.a0);
  y = 20 + (long)(b - cal.b0) * 200 / (cal.b1 - cal.b0);
  x = constrain(x, 0, SCR_W - 1); y = constrain(y, 0, SCR_H - 1);
  return true;
}
void drawCross(int x, int y) {
  tft.fillScreen(TFT_BLACK);
  tft.drawFastHLine(x - 10, y, 21, TFT_WHITE); tft.drawFastVLine(x, y - 10, 21, TFT_WHITE);
  tft.drawCircle(x, y, 6, TFT_RED);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.drawString("Touch the cross", 100, 112, 2);
}
void waitTap(int& rx, int& ry) {
  while (!readRaw(rx, ry)) delay(10);
  long sx = 0, sy = 0; int n = 0;
  for (int i = 0; i < 10; i++) { int a, b; if (readRaw(a, b)) { sx += a; sy += b; n++; } delay(10); }
  if (n) { rx = sx / n; ry = sy / n; }
  while (digitalRead(T_IRQ) == LOW) delay(10);
  delay(250);
}
void calibrate() {
  for (;;) {
    int ax, ay, bx, by;
    drawCross(20, 20); waitTap(ax, ay);
    drawCross(300, 220); waitTap(bx, by);
    int dx = bx - ax, dy = by - ay;
    if (abs(dx) < 800 || abs(dy) < 800) { msg("Try again", "Tap each cross firmly"); delay(1200); continue; }
    cal.swap = abs(dy) > abs(dx);
    if (!cal.swap) { cal.a0 = ax; cal.a1 = bx; cal.b0 = ay; cal.b1 = by; }
    else           { cal.a0 = ay; cal.a1 = by; cal.b0 = ax; cal.b1 = bx; }
    cal.valid = true; saveCal(); break;
  }
}

// ---- stations ----
void upsert(const Station& s) {
  int slot = -1; uint32_t oldest = 0xFFFFFFFF;
  for (int i = 0; i < MAX_ST; i++) {
    if (st[i].used && !strcmp(st[i].call, s.call)) { slot = i; break; }
    if (!st[i].used) { if (slot < 0 || st[slot].used) slot = i; }
    else if (st[i].seen < oldest && (slot < 0 || st[slot].used)) { oldest = st[i].seen; slot = i; }
  }
  st[slot] = s; st[slot].used = true; st[slot].seen = millis();
  dirty = true;
}

// ---- drawing ----
void drawBackground() {
  if (haveMap) {
    File f = SD.open("/map.raw");
    if (f) {
      static uint16_t src[SCR_W], line[SCR_W];
      int sw = SCR_W / zf, sh = SCR_H / zf;
      int sx0 = constrain((int)lround((vx0 - wx0) / worldW * SCR_W), 0, SCR_W - sw);
      int sy0 = constrain((int)lround((vy0 - wy0) / worldH * SCR_H), 0, SCR_H - sh);
      tft.setSwapBytes(true);
      for (int y = 0; y < SCR_H; y += zf) {
        f.seek((uint32_t)((sy0 + y / zf) * SCR_W + sx0) * 2);
        f.read((uint8_t*)src, sw * 2);
        for (int x = 0; x < SCR_W; x++) line[x] = src[x / zf];
        for (int k = 0; k < zf && y + k < SCR_H; k++) tft.pushImage(0, y + k, SCR_W, 1, line);
      }
      f.close(); return;
    }
  }
  tft.fillScreen(TFT_BLACK);
  for (int x = 0; x < SCR_W; x += 40) tft.drawFastVLine(x, 0, SCR_H, 0x18E3);
  for (int y = 0; y < SCR_H; y += 40) tft.drawFastHLine(0, y, SCR_W, 0x18E3);
  int hx, hy; project(homeLat, homeLon, hx, hy);
  tft.drawCircle(hx, hy, SCR_W / 4 * zf, 0x2965);
  tft.drawCircle(hx, hy, (SCR_W / 2 - 1) * zf, 0x2965);
}

void drawStatus() {
  char b[72]; int n = 0;
  for (auto& s : st) if (s.used && millis() - s.seen < EXPIRE_MS) n++;
  if (zf > 1) snprintf(b, sizeof(b), " %s  %d stn  %dx  %s ", cCall, n, zf, aprsOk ? "APRS-IS ok" : "connecting");
  else snprintf(b, sizeof(b), " %s  %d stn  %s ", cCall, n, aprsOk ? "APRS-IS ok" : "connecting");
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.drawString(b, 0, 0, 1);
}

void drawPopup() {
  if (selIdx < 0 || !st[selIdx].used) { selIdx = -1; return; }
  Station& s = st[selIdx];
  int x, y; if (project(s.lat, s.lon, x, y)) tft.drawCircle(x, y, 7, TFT_WHITE);
  const int py = SCR_H - 44;
  tft.fillRect(0, py, SCR_W, 44, TFT_BLACK); tft.drawRect(0, py, SCR_W, 44, TFT_DARKGREY);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK); tft.drawString(s.call, 6, py + 3, 2);
  char b[72];
  snprintf(b, sizeof(b), "%.4f  %.4f", s.lat, s.lon);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK); tft.drawString(b, 120, py + 7, 1);
  snprintf(b, sizeof(b), "%.1f km  %03.0f deg  %lu min ago", distKm(homeLat, homeLon, s.lat, s.lon),
           bearingDeg(homeLat, homeLon, s.lat, s.lon), (millis() - s.seen) / 60000UL);
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.drawString(b, 6, py + 23, 2);
}

void drawMap() {
  drawBackground();
  int hx, hy; project(homeLat, homeLon, hx, hy);
  tft.drawFastHLine(hx - 5, hy, 11, TFT_CYAN); tft.drawFastVLine(hx, hy - 5, 11, TFT_CYAN);
  int labels = 0;
  tft.setTextColor(TFT_WHITE);
  for (auto& s : st) {
    if (!s.used) continue;
    uint32_t age = millis() - s.seen;
    if (age > EXPIRE_MS) { s.used = false; continue; }
    int x, y; if (!project(s.lat, s.lon, x, y)) continue;
    uint16_t col = age < 300000 ? TFT_GREEN : age < 1800000 ? TFT_YELLOW : TFT_DARKGREY;
    tft.fillCircle(x, y, 3, col); tft.drawCircle(x, y, 3, TFT_BLACK);
    if (age < 600000 && labels < 14) { tft.drawString(s.call, x + 5, y - 4, 1); labels++; }
  }
  drawStatus();
  drawPopup();
}

void drawList() {
  tft.fillScreen(TFT_BLACK);
  int idx[MAX_ST], n = 0;
  for (int i = 0; i < MAX_ST; i++) if (st[i].used) idx[n++] = i;
  std::sort(idx, idx + n, [](int a, int b) { return st[a].seen > st[b].seen; });
  tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.drawString("CALL        km    age", 4, 2, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  for (int i = 0; i < n && i < 13; i++) {
    Station& s = st[idx[i]]; char b[48];
    snprintf(b, sizeof(b), "%-9s %4.0f  %3lum", s.call, distKm(homeLat, homeLon, s.lat, s.lon), (millis() - s.seen) / 60000UL);
    tft.drawString(b, 4, 20 + i * 16, 2);
  }
}

void handleTap(int x, int y) {
  if (listView) { listView = false; redraw(); return; }
  if (y < 14) { selIdx = -1; setView(wx0 + worldW / 2, wy0 + worldH / 2, 1); redraw(); return; }
  int best = -1, bd = 32 * 32;   // generous: fingers are imprecise
  for (int i = 0; i < MAX_ST; i++) {
    if (!st[i].used) continue;
    int sx, sy; project(st[i].lat, st[i].lon, sx, sy);
    int d2 = (sx - x) * (sx - x) + (sy - y) * (sy - y);
    if (d2 < bd) { bd = d2; best = i; }
  }
  if (best >= 0) { selIdx = best; redraw(); return; }
  if (selIdx >= 0) { selIdx = -1; redraw(); return; }
  if (zf < 4) {
    double wxT = vx0 + x / (double)SCR_W * vW, wyT = vy0 + y / (double)SCR_H * vH;
    setView(wxT, wyT, zf * 2);
  }
  redraw();
}

// ---- APRS-IS ----
bool connectAprs() {
  if (!client.connect(cServer, atoi(cPort), 5000)) return false;
  client.printf("user %s pass -1 vers CYDAPRS 0.2 filter r/%s/%s/%s\r\n", cCall, cLat, cLon, cRad);
  lastKeep = millis(); lineLen = 0;
  return true;
}
void pumpAprs() {
  int budget = 2048;
  while (budget-- > 0 && client.available()) {
    char c = client.read();
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      if (lineLen && lineBuf[lineLen - 1] == '\r') lineBuf[--lineLen] = 0;
      Station s;
      if (aprsParse(lineBuf, s)) { upsert(s); pktCount++; }
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
  }
}

// ---- setup / loop ----
bool saveFlag = false;
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(PIN_BL, OUTPUT); digitalWrite(PIN_BL, HIGH);
  tft.init(); tft.setRotation(1);
#ifdef CYD_FLIP_INVERT
  tft.invertDisplay(true);
#endif
  msg("CYD APRS map", "Starting...");
  loadCfg();
  touchInit();

  WiFiManager wm;
  uint32_t t0 = millis();
  while (digitalRead(PIN_BOOT) == LOW && millis() - t0 < 3000) delay(50);
  if (digitalRead(PIN_BOOT) == LOW) { wm.resetSettings(); msg("Settings reset", "Release BOOT"); delay(1500); }

  WiFiManagerParameter pCall("call", "Callsign", cCall, 11), pSrv("server", "APRS-IS server", cServer, 39),
      pPort("port", "Port", cPort, 7), pLat("lat", "Home latitude (e.g. 37.27)", cLat, 11),
      pLon("lon", "Home longitude (e.g. -79.94)", cLon, 11), pRad("rad", "Radius km (map half-width)", cRad, 7);
  WiFiManagerParameter* ps[] = {&pCall, &pSrv, &pPort, &pLat, &pLon, &pRad};
  for (auto* p : ps) wm.addParameter(p);
  wm.setSaveParamsCallback([]() { saveFlag = true; });
  wm.setAPCallback([](WiFiManager*) { msg("Setup: join Wi-Fi", "CYD-APRS-Setup -> 192.168.4.1"); });
  wm.setConfigPortalTimeout(300);
  if (!wm.autoConnect("CYD-APRS-Setup")) { delay(1000); ESP.restart(); }
  if (saveFlag) {
    strlcpy(cCall, pCall.getValue(), sizeof(cCall)); strlcpy(cServer, pSrv.getValue(), sizeof(cServer));
    strlcpy(cPort, pPort.getValue(), sizeof(cPort)); strlcpy(cLat, pLat.getValue(), sizeof(cLat));
    strlcpy(cLon, pLon.getValue(), sizeof(cLon)); strlcpy(cRad, pRad.getValue(), sizeof(cRad));
    for (char* c = cCall; *c; c++) *c = toupper(*c);
    saveCfg();
  }
  homeLat = atof(cLat); homeLon = atof(cLon); radiusKm = max(1.0, atof(cRad));
  setupProjection();

  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (SD.begin(SD_CS, sdSPI, 20000000)) {
    File f = SD.open("/map.raw");
    haveMap = f && f.size() == (size_t)SCR_W * SCR_H * 2;
    if (f) f.close();
  }
  if (!cal.valid) calibrate();
  msg(haveMap ? "Map found on SD" : "No /map.raw on SD", "Using grid fallback if missing");
  delay(800);
}

void loop() {
  static bool lastBtn = true, wasTouch = false;
  static uint32_t btnDown = 0, lastTap = 0;

  bool b = digitalRead(PIN_BOOT);
  if (!b && lastBtn) btnDown = millis();
  if (b && !lastBtn) {
    uint32_t held = millis() - btnDown;
    if (held > 1500) { calibrate(); redraw(); }
    else if (held > 40) { listView = !listView; redraw(); }
  }
  lastBtn = b;

  int tx, ty; bool t = touchScreen(tx, ty);
  if (t && !wasTouch && millis() - lastTap > 300) {
    lastTap = millis();
    // first contact is noisy: average a few samples once the finger has settled
    long sx = 0, sy = 0; int n = 0;
    for (int i = 0; i < 4; i++) { int ax, ay; delay(12); if (touchScreen(ax, ay)) { sx += ax; sy += ay; n++; } }
    if (n) { tx = sx / n; ty = sy / n; }
    handleTap(tx, ty);
  }
  wasTouch = t;

  if (!client.connected()) {
    aprsOk = false;
    if (millis() - lastTry > 5000) { lastTry = millis(); client.stop(); aprsOk = connectAprs(); dirty = true; }
  } else {
    aprsOk = true;
    pumpAprs();
    if (millis() - lastKeep > 20000) { client.print("#keepalive\r\n"); lastKeep = millis(); }
  }

  if ((dirty && millis() - lastDraw > 2000) || millis() - lastDraw > 15000) {
    listView ? drawList() : drawMap();
    lastDraw = millis(); dirty = false;
  }
  delay(5);
}
