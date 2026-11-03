// CYD APRS map: APRS-IS (read-only) -> 320x240 map with station dots.
// BOOT button: short press = toggle map/list. Hold at power-up = reset Wi-Fi/config.
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
double homeLat, homeLon, radiusKm, wx0, wy0, worldW, worldH;
char lineBuf[512]; size_t lineLen = 0;
uint32_t lastKeep = 0, lastTry = 0, lastDraw = 0, pktCount = 0;

// ---- projection (must match tools/make_map.py) ----
static double mercX(double lon) { return (lon + 180.0) / 360.0; }
static double mercY(double lat) { double s = sin(lat * DEG_TO_RAD); return 0.5 - log((1 + s) / (1 - s)) / (4 * PI); }
void setupProjection() {
  worldW = 2.0 * radiusKm * 1000.0 / (40075016.686 * cos(homeLat * DEG_TO_RAD));
  worldH = worldW * SCR_H / SCR_W;
  wx0 = mercX(homeLon) - worldW / 2; wy0 = mercY(homeLat) - worldH / 2;
}
bool project(double lat, double lon, int& x, int& y) {
  x = (int)((mercX(lon) - wx0) / worldW * SCR_W);
  y = (int)((mercY(lat) - wy0) / worldH * SCR_H);
  return x >= 0 && x < SCR_W && y >= 0 && y < SCR_H;
}
double distKm(double la1, double lo1, double la2, double lo2) {
  double dl = (la2 - la1) * DEG_TO_RAD, dn = (lo2 - lo1) * DEG_TO_RAD;
  double a = sin(dl / 2) * sin(dl / 2) + cos(la1 * DEG_TO_RAD) * cos(la2 * DEG_TO_RAD) * sin(dn / 2) * sin(dn / 2);
  return 12742.0 * asin(sqrt(a));
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
  prefs.end();
}
void saveCfg() {
  prefs.begin("aprs", false);
  prefs.putString("call", cCall); prefs.putString("server", cServer); prefs.putString("port", cPort);
  prefs.putString("lat", cLat); prefs.putString("lon", cLon); prefs.putString("rad", cRad);
  prefs.end();
}
void msg(const char* a, const char* b = "") {
  tft.fillScreen(TFT_BLACK); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(a, 8, 8, 2); tft.drawString(b, 8, 30, 2);
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
      static uint16_t buf[SCR_W * 8];
      tft.setSwapBytes(true);
      for (int y = 0; y < SCR_H; y += 8) {
        f.read((uint8_t*)buf, sizeof(buf));
        tft.pushImage(0, y, SCR_W, 8, buf);
      }
      f.close(); return;
    }
  }
  tft.fillScreen(TFT_BLACK);
  for (int x = 0; x < SCR_W; x += 40) tft.drawFastVLine(x, 0, SCR_H, 0x18E3);
  for (int y = 0; y < SCR_H; y += 40) tft.drawFastHLine(0, y, SCR_W, 0x18E3);
  tft.drawCircle(SCR_W / 2, SCR_H / 2, SCR_W / 4, 0x2965);
  tft.drawCircle(SCR_W / 2, SCR_H / 2, SCR_W / 2 - 1, 0x2965);
}

void drawStatus() {
  char b[64]; int n = 0;
  for (auto& s : st) if (s.used && millis() - s.seen < EXPIRE_MS) n++;
  snprintf(b, sizeof(b), " %s  %d stn  %s ", cCall, n, aprsOk ? "APRS-IS ok" : "connecting");
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.drawString(b, 0, 0, 1);
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

// ---- APRS-IS ----
bool connectAprs() {
  if (!client.connect(cServer, atoi(cPort), 5000)) return false;
  client.printf("user %s pass -1 vers CYDAPRS 0.1 filter r/%s/%s/%s\r\n", cCall, cLat, cLon, cRad);
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
  msg(haveMap ? "Map found on SD" : "No /map.raw on SD", "Using grid fallback if missing");
  delay(800);
}

void loop() {
  static bool lastBtn = true; static uint32_t btnT = 0;
  bool b = digitalRead(PIN_BOOT);
  if (!b && lastBtn && millis() - btnT > 250) { listView = !listView; dirty = true; btnT = millis(); }
  lastBtn = b;

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
