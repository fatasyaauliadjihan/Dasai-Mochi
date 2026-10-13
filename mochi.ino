/*
  Mochi Firmware v0.2  -  ESP32-C3 + OLED SSD1306 + MAX98357 + TTP223

  Kontrol:
    - Sensor sentuh (TTP223): HANYA untuk ekspresi, aktif di layar Wajah.
        tap = ketawa, tap 2x = marah, tahan = kesenengan (selama disentuh)
    - Push button: HANYA untuk navigasi & kontrol mode.
        double tap = pindah mode (Wajah -> Jam -> Timer -> Stopwatch -> Alarm -> Game -> ulang)
        tap        = aksi mode (start/stop stopwatch, start/pause timer, toggle alarm)
        tahan      = masuk MODE ATUR di Timer/Alarm, atau reset di Stopwatch

    Mode atur TIMER (masuk dengan tahan button di mode Timer):
        tap        = +1 menit
        tap 2x     = balik ke default (TIMER_PRESET_SEC)
        tahan      = simpan & keluar
        diam 4 detik = otomatis simpan & keluar

    Mode atur ALARM (masuk dengan tahan button di mode Alarm):
        tap        = +1 jam (kalau lagi atur jam) / +5 menit (kalau lagi atur menit)
        tap 2x     = pindah antara atur jam <-> atur menit
        tahan      = simpan & keluar
        diam 4 detik = otomatis simpan & keluar

  Mode:
    1. WAJAH     : ekspresi, dikontrol sensor sentuh
    2. JAM       : jam & tanggal dari WiFi (NTP)
    3. TIMER     : hitung mundur, preset TIMER_PRESET_SEC, bisa diatur dari button
    4. STOPWATCH : tap = start/stop, tahan = reset
    5. ALARM     : jam alarm ALARM_HOUR/ALARM_MIN, bisa diatur dari button
    6. GAME      : (belum ada, placeholder)

  Catatan: nilai timer & alarm hasil ATUR TIDAK disimpan ke flash,
  jadi balik ke default tiap device restart (sesuai permintaan).

  Library yang dibutuhkan (Library Manager):
    - U8g2 (by olikraus)
  Board package: "esp32" by Espressif versi 3.x  (butuh library ESP_I2S bawaan)
  Board: ESP32C3 Dev Module, USB CDC On Boot: Enabled
*/

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <time.h>
#include <ESP_I2S.h>
#include <esp_wifi.h>

// ====================== KONFIGURASI ======================
const char* WIFI_SSID = "Robotkidz";       // <-- ganti (harus WiFi 2.4GHz)
const char* WIFI_PASS = "robotkidz";       // <-- ganti
#define GMT_OFFSET_SEC (7 * 3600)          // WIB = UTC+7

#define PIN_SDA       20   // OLED
#define PIN_SCL       21
#define PIN_TOUCH     1    // TTP223 OUT
#define PIN_BTN       2    // push button, kaki satunya ke GND
#define PIN_I2S_BCLK  4    // MAX98357 BCLK
#define PIN_I2S_LRC   5    // MAX98357 LRC / WS
#define PIN_I2S_DIN   6    // MAX98357 DIN
// Jangan pakai GPIO0 untuk apa pun.

/* ====================== CATATAN WIRING ======================
   OLED SSD1306 (I2C)
     VCC  -> 3V3
     GND  -> GND
     SDA  -> GPIO20
     SCL  -> GPIO21

   Sensor sentuh TTP223
     VCC  -> 3V3
     GND  -> GND
     OUT  -> GPIO1   (HIGH saat disentuh)

   Push button
     Kaki 1 -> GPIO2
     Kaki 2 -> GND
     (pakai pull-up internal ESP32, jadi TIDAK perlu resistor tambahan;
      pin terbaca LOW saat tombol ditekan)

   MAX98357 (I2S audio amp)
     VIN  -> 3V3 atau 5V (ikuti modulnya, biasanya 5V lebih kencang)
     GND  -> GND
     BCLK -> GPIO4
     LRC / WS -> GPIO5
     DIN  -> GPIO6
     SD   -> biarkan lepas / tarik ke 3V3 (aktifkan speaker)
     GAIN -> biarkan lepas (gain default ~9dB), atau sambung ke GND
             untuk gain lebih besar, sesuai datasheet modulmu
     Speaker -> ke output + / - MAX98357 (bukan ke GPIO manapun)

   Pin yang DIHINDARI di ESP32-C3: GPIO0 (strapping boot), GPIO18/19
   kalau board-mu pakai untuk USB. Kalau salah satu pin di atas
   bentrok dengan board-mu, tinggal ganti angkanya di sini saja -
   seluruh kode lain memanggil lewat nama PIN_xxx ini.
   ============================================================= */

#define VOLUME        8000     // 0..32767, naikkan kalau kurang keras
#define IDLE_SOUND    false    // true = ekspresi acak saat idle ikut bunyi

#define LONG_MS       600      // tahan lebih dari ini = long press
#define DOUBLE_MS     300      // jeda maksimal antar tap untuk double tap
#define FRAME_MS      40       // ~25 fps

// Layar: SSD1306 0.96". Kalau layarmu 1.3" (biasanya SH1106), pakai baris kedua.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
// U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ====================== TIPE DATA (HARUS di atas) ======================
struct Note { uint16_t freq; uint16_t ms; };   // freq 0 = diam, ms 0 = akhir

enum SoundId {
  SND_HAPPY, SND_EXPR_BEEP, SND_SLEEPY, SND_SURPRISED, SND_ANGRY, SND_LOVE,
  SND_MODE, SND_START, SND_STOP, SND_RESET, SND_COUNT
};

enum Gesture { G_NONE, G_TAP, G_DOUBLE, G_LONG };
enum Mode    { M_FACE, M_CLOCK, M_TIMER, M_STOPWATCH, M_ALARM, M_GAME, M_COUNT };
enum Expr    { E_NORMAL, E_HAPPY, E_SLEEPY, E_SURPRISED, E_ANGRY, E_LOVE, E_COUNT };

// ====================== AUDIO ======================
#define SAMPLE_RATE  16000
#define AUDIO_CHUNK  256
#define ENV_SAMPLES  64

I2SClass i2s;
int16_t sineTable[256];
volatile int soundRequest = -1;

const Note S_HAPPY[]     = {{659,90},{784,90},{988,90},{1319,160},{0,0}};
const Note S_EXPR_BEEP[] = {{1400,80},{0,60},{1400,80},{0,0}};
const Note S_SLEEPY[]    = {{440,200},{392,200},{330,300},{262,400},{0,0}};
const Note S_SURPRISED[] = {{1600,70},{0,60},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{0,0}};
const Note S_ANGRY[]     = {{700,60},{450,110},{0,0}};
const Note S_LOVE[]      = {{784,100},{988,100},{1175,100},{1568,100},{1319,200},{0,0}};
const Note S_MODE[]      = {{700,80},{0,60},{700,80},{0,0}};
const Note S_START[]     = {{1047,80},{1568,120},{0,0}};
const Note S_STOP[]      = {{1600,70},{0,60},{1600,70},{0,60},{1600,70},{0,60},{1600,70},{0,0}};
const Note S_RESET[]     = {{784,60},{523,60},{392,120},{0,0}};

const Note* const SOUNDS[SND_COUNT] = {
  S_HAPPY, S_EXPR_BEEP, S_SLEEPY, S_SURPRISED, S_ANGRY, S_LOVE,
  S_MODE, S_START, S_STOP, S_RESET
};

void playSound(int id) { soundRequest = id; }

void audioTask(void*) {
  static int16_t buf[AUDIO_CHUNK];
  for (;;) {
    int req = soundRequest;
    if (req >= 0 && req < SND_COUNT) {
      soundRequest = -1;
      const Note* n = SOUNDS[req];
      uint32_t phase = 0;
      for (; n->ms && soundRequest < 0; n++) {
        uint32_t total = (uint32_t)SAMPLE_RATE * n->ms / 1000;
        uint32_t step = n->freq ? (uint32_t)(((uint64_t)n->freq << 32) / SAMPLE_RATE) : 0;
        uint32_t done = 0;
        while (done < total && soundRequest < 0) {
          uint32_t cnt = total - done;
          if (cnt > AUDIO_CHUNK) cnt = AUDIO_CHUNK;
          for (uint32_t i = 0; i < cnt; i++) {
            int32_t v = 0;
            if (step) {
              uint32_t pos = done + i;
              uint32_t edge = pos < (total - 1 - pos) ? pos : (total - 1 - pos);
              phase += step;
              v = sineTable[phase >> 24];
              if (edge < ENV_SAMPLES) v = v * (int32_t)edge / ENV_SAMPLES;
              v = v * ((req == SND_SURPRISED) ? 24000 : VOLUME) / 32767;
            }
            buf[i] = (int16_t)v;
          }
          i2s.write((uint8_t*)buf, cnt * 2);
          done += cnt;
        }
      }
    } else {
      memset(buf, 0, sizeof(buf));
      i2s.write((uint8_t*)buf, sizeof(buf));
    }
  }
}

void audioBegin() {
  for (int i = 0; i < 256; i++) sineTable[i] = (int16_t)(32767.0f * sinf(2.0f * PI * i / 256.0f));
  i2s.setPins(PIN_I2S_BCLK, PIN_I2S_LRC, PIN_I2S_DIN);
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
    Serial.println("I2S gagal start, cek pin MAX98357");
    return;
  }
  xTaskCreate(audioTask, "audio", 4096, NULL, 1, NULL);
}

// ====================== INPUT (sentuh & button, terpisah) ======================
Gesture readGestureFor(uint8_t idx, bool now) {
  static bool prev[2] = {false, false};
  static bool longFired[2] = {false, false};
  static uint32_t pressT[2] = {0, 0}, releaseT[2] = {0, 0};
  static uint8_t taps[2] = {0, 0};
  uint32_t t = millis();
  Gesture g = G_NONE;

  if (now && !prev[idx]) { pressT[idx] = t; longFired[idx] = false; }
  if (now && !longFired[idx] && (t - pressT[idx]) > LONG_MS) { longFired[idx] = true; taps[idx] = 0; g = G_LONG; }
  if (!now && prev[idx]) { releaseT[idx] = t; if (!longFired[idx]) taps[idx]++; }
  if (!now && taps[idx] > 0 && (t - releaseT[idx]) > DOUBLE_MS) {
    g = (taps[idx] >= 2) ? G_DOUBLE : G_TAP;
    taps[idx] = 0;
  }
  prev[idx] = now;
  return g;
}

Gesture readTouch()  { return readGestureFor(0, digitalRead(PIN_TOUCH) == HIGH); }
Gesture readButton() { return readGestureFor(1, digitalRead(PIN_BTN) == LOW); }

// ====================== MODE ======================
const char* MODE_NAMES[M_COUNT] = {"EXPRESSION", "CLOCK", "TIMER", "STOPWATCH", "ALARM", "GAME"};
Mode mode = M_FACE;
uint32_t bannerUntil = 0;

// ====================== EKSPRESI ======================
const int EXPR_SOUND[E_COUNT] = { -1, SND_HAPPY, SND_SLEEPY, SND_SURPRISED, SND_ANGRY, SND_LOVE };

Expr expr = E_NORMAL;
uint32_t exprUntil = 0;
bool petting = false;
float lookX = 0, lookTarget = 0;
uint32_t nextLook = 0, nextBlink = 2000, blinkEnd = 0, nextIdleExpr = 15000;

const int EYE_LX = 36, EYE_RX = 92, EY = 26;

void setExpr(Expr e, uint32_t holdMs, bool withSound) {
  expr = e;
  exprUntil = holdMs ? millis() + holdMs : 0;
  if (withSound) playSound(SND_EXPR_BEEP);
}

void updateEyes() {
  uint32_t t = millis();
  if (petting) return;

  if (exprUntil && t > exprUntil) { expr = E_NORMAL; exprUntil = 0; }

  if (expr == E_NORMAL && t > nextIdleExpr) {
    const Expr pool[] = {E_HAPPY, E_SURPRISED, E_LOVE};
    setExpr(pool[random(3)], 2500, IDLE_SOUND);
    nextIdleExpr = t + random(15000, 35000);
  }
  if (t > nextLook) {
    lookTarget = (float)random(-8, 9);
    nextLook = t + random(1500, 4000);
  }
  lookX += (lookTarget - lookX) * 0.25f;
  if (t > nextBlink) { blinkEnd = t + 140; nextBlink = t + random(2000, 5000); }
}

void drawHeart(int cx, int cy, int r) {
  u8g2.drawDisc(cx - r, cy - r / 2, r);
  u8g2.drawDisc(cx + r, cy - r / 2, r);
  u8g2.drawTriangle(cx - 2 * r + 1, cy - r / 2 + 2, cx + 2 * r - 1, cy - r / 2 + 2, cx, cy + 2 * r);
}

void drawEyes() {
  uint32_t t = millis();
  int ox = (int)lookX;
  bool blinking = t < blinkEnd;
  int xs[2] = {EYE_LX + ox, EYE_RX + ox};

  switch (expr) {
    case E_NORMAL: {
      int h = blinking ? 4 : 34, w = 28;
      int r = h / 2 < 8 ? h / 2 : 8;
      for (int i = 0; i < 2; i++) u8g2.drawRBox(xs[i] - w / 2, EY - h / 2, w, h, r);
      u8g2.drawCircle(64, 46, 6, U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
      break;
    }
    case E_HAPPY:
      for (int i = 0; i < 2; i++)
        for (int r = 12; r <= 14; r++)
          u8g2.drawCircle(xs[i], EY + 6, r, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
      u8g2.drawDisc(64, 44, 9, U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
      break;
    case E_SLEEPY:
      for (int i = 0; i < 2; i++) u8g2.drawRBox(xs[i] - 14, EY + 6, 28, 5, 2);
      u8g2.drawCircle(64, 50, 3);
      u8g2.setFont(u8g2_font_6x12_tr);
      u8g2.drawStr(104, 16, "z");
      if ((t / 600) % 2) u8g2.drawStr(112, 8, "Z");
      break;
    case E_SURPRISED:
      for (int i = 0; i < 2; i++) {
        u8g2.drawCircle(xs[i], EY, 15);
        u8g2.drawDisc(xs[i] + ox / 2, EY, 5);
      }
      u8g2.drawCircle(64, 50, 6);
      break;
    case E_ANGRY:
      for (int i = 0; i < 2; i++) u8g2.drawRBox(xs[i] - 14, EY - 16, 28, 32, 6);
      u8g2.setDrawColor(0);
      u8g2.drawTriangle(xs[0] - 15, EY - 17, xs[0] + 15, EY - 17, xs[0] + 15, EY - 1);
      u8g2.drawTriangle(xs[1] - 15, EY - 17, xs[1] + 15, EY - 17, xs[1] - 15, EY - 1);
      u8g2.setDrawColor(1);
      u8g2.drawCircle(64, 54, 8, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
      break;
    case E_LOVE: {
      int r = 7 + ((t / 250) % 2);
      for (int i = 0; i < 2; i++) drawHeart(xs[i], EY, r);
      u8g2.drawDisc(64, 46, 6, U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
      break;
    }
    default: break;
  }
}

void exprGesture(Gesture g) {
  if (g == G_TAP) {
    setExpr(E_HAPPY, 2000, true);
  } else if (g == G_DOUBLE) {
    setExpr(E_ANGRY, 2500, true);
  }
}

// ====================== JAM ======================
bool ntpStarted = false;

void wifiTick() {
  if (!ntpStarted && WiFi.status() == WL_CONNECTED) {
    configTime(GMT_OFFSET_SEC, 0, "id.pool.ntp.org", "time.google.com");
    ntpStarted = true;
  }
}

void drawCentered(const char* s, int y) {
  int w = u8g2.getStrWidth(s);
  u8g2.drawStr((128 - w) / 2, y, s);
}

void drawClock() {
  struct tm ti;
  if (getLocalTime(&ti, 0)) {
    char hm[8], d[24];
    snprintf(hm, sizeof(hm), "%02d:%02d", ti.tm_hour, ti.tm_min);
    strftime(d, sizeof(d), "%a, %d %b %Y", &ti);
    u8g2.setFont(u8g2_font_logisoso32_tn);
    drawCentered(hm, 44);
    u8g2.setFont(u8g2_font_6x12_tr);
    drawCentered(d, 62);
  } else {
    u8g2.setFont(u8g2_font_6x12_tr);
    drawCentered(WiFi.status() == WL_CONNECTED ? "Sinkron waktu..." : "Menunggu WiFi...", 36);
  }
}

// ====================== STOPWATCH ======================
bool swRunning = false;
uint32_t swStart = 0, swElapsed = 0;

uint32_t swNow() { return swElapsed + (swRunning ? millis() - swStart : 0); }

void swToggle() {
  if (swRunning) { swElapsed += millis() - swStart; swRunning = false; playSound(SND_STOP); }
  else           { swStart = millis(); swRunning = true; playSound(SND_START); }
}
void swReset() { swRunning = false; swElapsed = 0; playSound(SND_RESET); }

void drawStopwatch() {
  uint32_t ms = swNow();
  char a[8], b[8];
  snprintf(a, sizeof(a), "%02lu:%02lu", (unsigned long)(ms / 60000 % 100), (unsigned long)(ms / 1000 % 60));
  snprintf(b, sizeof(b), ".%02lu", (unsigned long)(ms / 10 % 100));
  u8g2.setFont(u8g2_font_logisoso32_tn);
  drawCentered(a, 40);
  u8g2.setFont(u8g2_font_helvB14_tr);
  drawCentered(b, 62);
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 62, swRunning ? "RUN" : "STOP");
}

// ====================== TIMER ======================
#define TIMER_PRESET_SEC 1   // 1 detik, ganti sesuai selera
#define TIMER_STEP_SEC 1
#define ADJUST_TIMEOUT_MS 4000

bool timerRunning = false;
uint32_t timerStartMs = 0;
int32_t timerRemainMs = TIMER_PRESET_SEC * 1000;
bool timerRang = false;

bool timerAdjusting = false;
uint32_t timerAdjustLastTap = 0;

int32_t timerNowMs() {
  if (!timerRunning) return timerRemainMs;
  int32_t left = timerRemainMs - (int32_t)(millis() - timerStartMs);
  return left > 0 ? left : 0;
}

void timerToggle() {
  if (timerRang) { timerRang = false; timerRemainMs = TIMER_PRESET_SEC * 1000; playSound(SND_RESET); return; }
  if (timerRunning) { timerRemainMs = timerNowMs(); timerRunning = false; playSound(SND_STOP); }
  else              { timerStartMs = millis(); timerRunning = true; playSound(SND_START); }
}

void timerAdjustEnter()   { timerAdjusting = true; timerRunning = false; timerRang = false; timerAdjustLastTap = millis(); playSound(SND_MODE); }
void timerAdjustTap()     { timerRemainMs += TIMER_STEP_SEC * 1000L; if (timerRemainMs > 60L*60*1000) timerRemainMs = TIMER_STEP_SEC*1000L; timerAdjustLastTap = millis(); playSound(SND_START); }
void timerAdjustDefault() { timerRemainMs = TIMER_PRESET_SEC * 1000; timerAdjustLastTap = millis(); playSound(SND_RESET); }
void timerAdjustExit()    { timerAdjusting = false; playSound(SND_STOP); }
void timerAdjustTick()    { if (timerAdjusting && millis() - timerAdjustLastTap > ADJUST_TIMEOUT_MS) timerAdjustExit(); }

void timerTick() {
  if (timerRunning && !timerRang && timerNowMs() == 0) {
    timerRunning = false;
    timerRang = true;
    playSound(SND_SURPRISED);
  }
}

void drawTimer() {
  int32_t ms = timerNowMs();
  char a[8];
  snprintf(a, sizeof(a), "%02ld:%02ld", (long)(ms / 60000), (long)(ms / 1000 % 60));
  u8g2.setFont(u8g2_font_logisoso32_tn);
  drawCentered(a, 44);
  u8g2.setFont(u8g2_font_6x12_tr);
  if (timerAdjusting)   drawCentered("ATUR: tap +1mnt", 62);
  else if (timerRang)   drawCentered("SELESAI! tap=reset", 62);
  else                  drawCentered(timerRunning ? "RUN" : "PAUSE (tahan=atur)", 62);
}

// ====================== ALARM ======================
int ALARM_HOUR = 0, ALARM_MIN = 00;   // ganti sesuai kebutuhan
#define ALARM_MIN_STEP 5

bool alarmEnabled = false;
bool alarmRinging = false;
int lastCheckedMinute = -1;

bool alarmAdjusting = false;
uint8_t alarmAdjustField = 0;   // 0 = jam, 1 = menit
uint32_t alarmAdjustLastTap = 0;

void alarmToggle() {
  if (alarmRinging) { alarmRinging = false; playSound(SND_STOP); return; }
  alarmEnabled = !alarmEnabled;
  playSound(alarmEnabled ? SND_START : SND_STOP);
}

void alarmAdjustEnter()  { alarmAdjusting = true; alarmAdjustField = 0; alarmRinging = false; alarmAdjustLastTap = millis(); playSound(SND_MODE); }
void alarmAdjustTap()    { if (alarmAdjustField == 0) ALARM_HOUR = (ALARM_HOUR + 1) % 24; else ALARM_MIN = (ALARM_MIN + ALARM_MIN_STEP) % 60; alarmAdjustLastTap = millis(); playSound(SND_START); }
void alarmAdjustSwitch() { alarmAdjustField = 1 - alarmAdjustField; alarmAdjustLastTap = millis(); playSound(SND_MODE); }
void alarmAdjustExit()   { alarmAdjusting = false; playSound(SND_STOP); }
void alarmAdjustTick()   { if (alarmAdjusting && millis() - alarmAdjustLastTap > ADJUST_TIMEOUT_MS) alarmAdjustExit(); }

void alarmTick() {
  struct tm ti;
  if (!alarmEnabled || alarmRinging || !getLocalTime(&ti, 0)) return;
  if (ti.tm_min != lastCheckedMinute) {
    lastCheckedMinute = ti.tm_min;
    if (ti.tm_hour == ALARM_HOUR && ti.tm_min == ALARM_MIN) {
      alarmRinging = true;
      playSound(SND_SURPRISED);
    }
  }
}

void drawAlarm() {
  char a[8];
  snprintf(a, sizeof(a), "%02d:%02d", ALARM_HOUR, ALARM_MIN);
  u8g2.setFont(u8g2_font_logisoso32_tn);
  drawCentered(a, 44);
  u8g2.setFont(u8g2_font_6x12_tr);
  if (alarmAdjusting) drawCentered(alarmAdjustField == 0 ? "ATUR JAM: tap +1" : "ATUR MENIT: tap +5", 62);
  else if (alarmRinging) drawCentered("BERDERING! tap=stop", 62);
  else drawCentered(alarmEnabled ? "AKTIF (tahan=atur)" : "MATI (tahan=atur)", 62);
}

// ====================== GAME (placeholder) ======================
// ====================== GAME ======================
#define GROUND_Y   56
#define MOCHI_X    18
#define MOCHI_W    14
#define MOCHI_H    14
#define GS_READY   0
#define GS_PLAY    1
#define GS_OVER    2

const float GAME_JUMP_V   = 8.6f;   // makin besar = lompat makin tinggi
const float GAME_GRAVITY  = 1.1f;   // makin besar = jatuh makin cepat

uint8_t  gState = GS_READY;
int      gScore = 0, gHigh = 0;
float    gY = 0, gVy = 0;           // gY = tinggi Mochi dari tanah
float    gSpeed = 3.5f, gNextSpawn = 70, gScroll = 0;
uint32_t gDeadAt = 0;
bool     gPrevTouch = false, gTouchEdge = false;

float   obsX[3];
uint8_t obsW[3], obsH[3];
bool    obsOn[3], obsScored[3];

void gameReset() {                  // dipanggil tiap masuk mode game
  gState = GS_READY;
  gY = 0; gVy = 0; gScroll = 0;
  for (int i = 0; i < 3; i++) obsOn[i] = false;
  gPrevTouch = digitalRead(PIN_TOUCH) == HIGH;
  gTouchEdge = false;
}

void gameStart() {
  gState = GS_PLAY;
  gScore = 0; gY = 0; gVy = 0;
  gSpeed = 3.5f; gNextSpawn = 70; gScroll = 0;
  for (int i = 0; i < 3; i++) obsOn[i] = false;
  playSound(SND_MODE);
}

// tap push button = mulai / main lagi (ada jeda 0,7 detik setelah kalah)
void gameButtonStart() {
  if (gState == GS_READY || (gState == GS_OVER && millis() - gDeadAt > 700)) gameStart();
}

// dipanggil tiap loop() supaya tap singkat nggak kelewat
void gamePollInput() {
  bool t = digitalRead(PIN_TOUCH) == HIGH;
  if (t && !gPrevTouch) gTouchEdge = true;
  gPrevTouch = t;
}

void gameUpdate() {
  bool pressed = gTouchEdge;
  gTouchEdge = false;

  if (gState != GS_PLAY) {          // sentuh juga bisa buat mulai / main lagi
    if (pressed) gameButtonStart();
    return;
  }

  // lompat + fisika
  if (pressed && gY < 0.5f) { gVy = GAME_JUMP_V; playSound(SND_START); }
  gY += gVy;
  gVy -= GAME_GRAVITY;
  if (gY <= 0) { gY = 0; gVy = 0; }

  // rintangan baru
  gScroll += gSpeed;
  gNextSpawn -= gSpeed;
  if (gNextSpawn <= 0) {
    for (int i = 0; i < 3; i++) {
      if (!obsOn[i]) {
        obsOn[i] = true; obsX[i] = 128;
        obsW[i] = random(7, 13); obsH[i] = random(10, 21);
        obsScored[i] = false;
        break;
      }
    }
    gNextSpawn = random(60, 111) + gSpeed * 8;   // jarak antar rintangan
  }

  // gerak, skor, tabrakan
  for (int i = 0; i < 3; i++) {
    if (!obsOn[i]) continue;
    obsX[i] -= gSpeed;
    if (obsX[i] + obsW[i] < 0) { obsOn[i] = false; continue; }

    if (!obsScored[i] && obsX[i] + obsW[i] < MOCHI_X) {
      obsScored[i] = true;
      gScore++;
      if (gSpeed < 9.0f) gSpeed += 0.15f;        // makin lama makin cepat
      if (gScore % 10 == 0) playSound(SND_HAPPY);
    }

    if (obsX[i] < MOCHI_X + MOCHI_W - 2 && obsX[i] + obsW[i] > MOCHI_X + 2 && gY < obsH[i] - 2) {
      gState = GS_OVER;
      gDeadAt = millis();
      if (gScore > gHigh) gHigh = gScore;
      playSound(SND_RESET);
      return;
    }
  }
}

void drawGame() {
  gameUpdate();

  // tanah (garis putus-putus ikut bergerak)
  u8g2.drawHLine(0, GROUND_Y, 128);
  int off = (int)gScroll % 16;
  for (int x = -off; x < 128; x += 16) if (x >= 0) u8g2.drawHLine(x, GROUND_Y + 3, 3);

  // rintangan
  for (int i = 0; i < 3; i++)
    if (obsOn[i]) u8g2.drawBox((int)obsX[i], GROUND_Y - obsH[i], obsW[i], obsH[i]);

  // Mochi
  int my = GROUND_Y - (int)gY - MOCHI_H;
  u8g2.drawRBox(MOCHI_X, my, MOCHI_W, MOCHI_H, 4);
  u8g2.setDrawColor(0);
  if (gState == GS_OVER) {          // mata X pas kalah
    u8g2.drawLine(MOCHI_X + 3, my + 4, MOCHI_X + 5, my + 6);
    u8g2.drawLine(MOCHI_X + 5, my + 4, MOCHI_X + 3, my + 6);
    u8g2.drawLine(MOCHI_X + 8, my + 4, MOCHI_X + 10, my + 6);
    u8g2.drawLine(MOCHI_X + 10, my + 4, MOCHI_X + 8, my + 6);
  } else {
    u8g2.drawBox(MOCHI_X + 4, my + 4, 2, 3);
    u8g2.drawBox(MOCHI_X + 9, my + 4, 2, 3);
  }
  u8g2.setDrawColor(1);

  // tulisan
  char s[24];
  u8g2.setFont(u8g2_font_6x12_tr);
  if (gState == GS_READY) {
    u8g2.setFont(u8g2_font_helvB14_tr);
    drawCentered("MOCHI RUN", 24);
    u8g2.setFont(u8g2_font_6x12_tr);
    drawCentered("sentuh = mulai", 38);
    if (gHigh > 0) { snprintf(s, sizeof(s), "HI %d", gHigh); u8g2.drawStr(128 - u8g2.getStrWidth(s) - 1, 10, s); }
  } else {
    snprintf(s, sizeof(s), "HI %d  %d", gHigh, gScore);
    u8g2.drawStr(128 - u8g2.getStrWidth(s) - 1, 10, s);
    if (gState == GS_OVER) {
      u8g2.setFont(u8g2_font_helvB14_tr);
      drawCentered("GAME OVER", 28);
      u8g2.setFont(u8g2_font_6x12_tr);
      drawCentered("sentuh = lagi", 40);
    }
  }
}

// ====================== UTAMA ======================
void nextMode() {
  mode = (Mode)((mode + 1) % M_COUNT);
  if (mode == M_GAME) gameReset();  
  bannerUntil = millis() + 1200;
  playSound(SND_MODE);
}

void render() {
  u8g2.clearBuffer();
  switch (mode) {
    case M_FACE:      updateEyes(); drawEyes(); break;
    case M_CLOCK:     drawClock(); break;
    case M_TIMER:     drawTimer(); break;
    case M_STOPWATCH: drawStopwatch(); break;
    case M_ALARM:     drawAlarm(); break;
    case M_GAME:      drawGame(); break;
    default: break;
  }
  if (millis() < bannerUntil) {
    u8g2.setFont(u8g2_font_6x12_tr);
    int w = u8g2.getStrWidth(MODE_NAMES[mode]) + 6;
    u8g2.setDrawColor(0);
    u8g2.drawBox(0, 0, w + 2, 14);
    u8g2.setDrawColor(1);
    u8g2.drawFrame(0, 0, w + 2, 14);
    u8g2.drawStr(4, 11, MODE_NAMES[mode]);
  }
  u8g2.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TOUCH, INPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  randomSeed(esp_random());

  Wire.setPins(PIN_SDA, PIN_SCL);
  u8g2.begin();
  u8g2.setBusClock(400000);

  audioBegin();
     int n = WiFi.scanNetworks();
   for (int i = 0; i < n; i++) {
     Serial.printf("%d: %s (%d dBm)\n", i, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
   }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Menyambung WiFi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(300);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" Tersambung!");
    Serial.println(WiFi.localIP());
  } else {
  Serial.printf(" GAGAL connect. Kode status: %d\n", WiFi.status());
  }

  setExpr(E_HAPPY, 2000, true);
}

void loop() {
  static uint32_t lastFrame = 0;

  wifiTick();
  timerTick();
  alarmTick();
  timerAdjustTick();
  alarmAdjustTick();
  if (mode == M_GAME) gamePollInput();

  // --- sensor sentuh: HANYA ekspresi, hanya berlaku di mode WAJAH ---
  bool touchNow = digitalRead(PIN_TOUCH) == HIGH;
  Gesture gt = readTouch();
  if (mode == M_FACE) {
    if (gt == G_LONG) { petting = true; setExpr(E_LOVE, 0, true); }
    if (petting && !touchNow) { petting = false; setExpr(E_NORMAL, 0, false); }
    if (!petting) {
      if (gt == G_TAP) exprGesture(gt);
      else if (gt == G_DOUBLE) exprGesture(gt);
    }
  }

  // --- push button: navigasi mode, kontrol mode, atau mode atur ---
  Gesture gb = readButton();
  if (mode == M_TIMER && timerAdjusting) {
    if (gb == G_TAP) timerAdjustTap();
    else if (gb == G_DOUBLE) timerAdjustDefault();
    else if (gb == G_LONG) timerAdjustExit();
  } else if (mode == M_ALARM && alarmAdjusting) {
    if (gb == G_TAP) alarmAdjustTap();
    else if (gb == G_DOUBLE) alarmAdjustSwitch();
    else if (gb == G_LONG) alarmAdjustExit();
  } else {
    if (gb == G_DOUBLE) {
      nextMode();
    } else if (gb == G_TAP) {
      if      (mode == M_STOPWATCH) swToggle();
      else if (mode == M_TIMER)     timerToggle();
      else if (mode == M_ALARM)     alarmToggle();
      else if (mode == M_GAME)      gameButtonStart();
    } else if (gb == G_LONG) {
      if      (mode == M_STOPWATCH) swReset();
      else if (mode == M_TIMER)     timerAdjustEnter();
      else if (mode == M_ALARM)     alarmAdjustEnter();
    }
  }

  if (millis() - lastFrame >= FRAME_MS) {
    lastFrame = millis();
    render();
  }
}