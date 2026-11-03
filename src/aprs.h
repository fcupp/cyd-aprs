#pragma once
#include <Arduino.h>

struct Station {
  char call[10];
  float lat, lon;
  uint32_t seen;   // millis() of last packet
  char table, code;
  bool used;
};

static inline int aprsDig(char c) { return c == ' ' ? 0 : c - '0'; }
static inline bool aprsIsDig(char c) { return c == ' ' || (c >= '0' && c <= '9'); }

// DDMM.mmN/DDDMM.mmW$
static bool aprsUncompressed(const char* p, double& lat, double& lon, char& tb, char& cd) {
  if (strlen(p) < 19) return false;
  const int digs[] = {0, 1, 2, 3, 5, 6, 9, 10, 11, 12, 13, 15, 16};
  for (int i : digs)
    if (!aprsIsDig(p[i])) return false;
  if (p[4] != '.' || p[14] != '.') return false;
  lat = aprsDig(p[0]) * 10 + aprsDig(p[1]) +
        (aprsDig(p[2]) * 10 + aprsDig(p[3]) + (aprsDig(p[5]) * 10 + aprsDig(p[6])) / 100.0) / 60.0;
  if (p[7] == 'S') lat = -lat; else if (p[7] != 'N') return false;
  lon = aprsDig(p[9]) * 100 + aprsDig(p[10]) * 10 + aprsDig(p[11]) +
        (aprsDig(p[12]) * 10 + aprsDig(p[13]) + (aprsDig(p[15]) * 10 + aprsDig(p[16])) / 100.0) / 60.0;
  if (p[17] == 'W') lon = -lon; else if (p[17] != 'E') return false;
  tb = p[8]; cd = p[18];
  return true;
}

// T YYYY XXXX C cs T  (base-91)
static bool aprsCompressed(const char* p, double& lat, double& lon, char& tb, char& cd) {
  if (strlen(p) < 13) return false;
  long y = 0, x = 0;
  for (int i = 0; i < 4; i++) {
    int a = p[1 + i] - 33, b = p[5 + i] - 33;
    if (a < 0 || a > 90 || b < 0 || b > 90) return false;
    y = y * 91 + a; x = x * 91 + b;
  }
  lat = 90.0 - y / 380926.0;
  lon = -180.0 + x / 190463.0;
  tb = p[0]; cd = p[9];
  return true;
}

static bool aprsPos(const char* p, double& lat, double& lon, char& tb, char& cd) {
  return aprsIsDig(p[0]) ? aprsUncompressed(p, lat, lon, tb, cd) : aprsCompressed(p, lat, lon, tb, cd);
}

static bool aprsMicE(const char* dest, const char* info, double& lat, double& lon, char& tb, char& cd) {
  if (strlen(dest) < 6 || strlen(info) < 9) return false;
  int d[6];
  for (int i = 0; i < 6; i++) {
    char c = dest[i];
    if (c >= '0' && c <= '9') d[i] = c - '0';
    else if (c >= 'A' && c <= 'J') d[i] = c - 'A';
    else if (c >= 'P' && c <= 'Y') d[i] = c - 'P';
    else if (c == 'K' || c == 'L' || c == 'Z') d[i] = 0;
    else return false;
  }
  lat = d[0] * 10 + d[1] + ((d[2] * 10 + d[3]) + (d[4] * 10 + d[5]) / 100.0) / 60.0;
  if (dest[3] < 'P') lat = -lat;
  int ld = (uint8_t)info[1] - 28;
  if (dest[4] >= 'P') ld += 100;
  if (ld >= 180 && ld <= 189) ld -= 80; else if (ld >= 190 && ld <= 199) ld -= 190;
  int lm = (uint8_t)info[2] - 28; if (lm >= 60) lm -= 60;
  int lh = (uint8_t)info[3] - 28;
  lon = ld + (lm + lh / 100.0) / 60.0;
  if (dest[5] >= 'P') lon = -lon;
  cd = info[7]; tb = info[8];
  return true;
}

// Parses one APRS-IS line. Returns true if a position was found.
static bool aprsParse(const char* line, Station& out) {
  if (line[0] == '#' || !line[0]) return false;
  const char* gt = strchr(line, '>');
  const char* colon = strchr(line, ':');
  if (!gt || !colon || gt > colon) return false;
  size_t n = gt - line;
  if (n == 0 || n >= sizeof(out.call)) return false;
  memcpy(out.call, line, n); out.call[n] = 0;
  char dest[8] = {0}; size_t i = 0;
  for (const char* d = gt + 1; *d && *d != ',' && *d != ':' && i < 7; d++) dest[i++] = *d;
  const char* p = colon + 1;
  double lat = 0, lon = 0; char tb = '/', cd = '.'; bool ok = false;
  switch (*p) {
    case '!': case '=': ok = aprsPos(p + 1, lat, lon, tb, cd); break;
    case '/': case '@': if (strlen(p) > 8) ok = aprsPos(p + 8, lat, lon, tb, cd); break;
    case ';':  // object: ;NAME_____*DDHHMMzPOS
      if (strlen(p) > 18 && p[10] == '*') {
        int k = 9; while (k > 1 && p[k] == ' ') k--;
        size_t m = (size_t)k < sizeof(out.call) - 1 ? (size_t)k : sizeof(out.call) - 1;
        memcpy(out.call, p + 1, m); out.call[m] = 0;
        ok = aprsPos(p + 18, lat, lon, tb, cd);
      }
      break;
    case '`': case '\'': ok = aprsMicE(dest, p, lat, lon, tb, cd); break;
  }
  if (!ok || lat < -90 || lat > 90 || lon < -180 || lon > 180 || (lat == 0 && lon == 0)) return false;
  out.lat = lat; out.lon = lon; out.table = tb; out.code = cd;
  return true;
}
