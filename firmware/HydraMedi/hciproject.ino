#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>
#include <HX711.h>
#include <Preferences.h>
#include <math.h>

// -----------------------------------------------------------------------
// CONFIG
// -----------------------------------------------------------------------
const char* WIFI_SSID = "--- Feni";
const char* WIFI_PASS = "12345678@@";
const char* FIREBASE_URL = "https://hydramedi-default-rtdb.firebaseio.com";

// PINS
#define PIN_SDA 21
#define PIN_SCL 22
#define PIN_HX_DAT 4
#define PIN_HX_CLK 5

// Corrected to match your wiring comment: R=25, G=26, B=27
#define PIN_RGB_R 25
#define PIN_RGB_G 26
#define PIN_RGB_B 27

#define PIN_BUZZER 14
#define PIN_BUTTON 13
#define PIN_SERVO1 18
#define PIN_SERVO2 19
#define PIN_SERVO3 23

// New medicine timing LEDs
#define PIN_LED_BEFORE 32
#define PIN_LED_AFTER 33

// RGB LED TYPE
#define RGB_COMMON_ANODE false

// TIMING
#define MAX_MEDS 3
#define BUZZER_DURATION_MS 20000UL
#define SNOOZE_INTERVAL_MS (30UL * 60UL * 1000UL)
#define DISPLAY_REFRESH_MS 1000UL
#define REMINDER_DISPLAY_MS 500UL
#define WATER_DISPLAY_MS 250UL
#define FIREBASE_SYNC_MS 20000UL
#define DEBOUNCE_MS 200UL

// HX711 calibration - keep your existing value initially, then recalibrate
#define HX711_CALIBRATION -5.214126f

// -----------------------------------------------------------------------
// WEIGHT FILTER / STABILITY SETTINGS
// -----------------------------------------------------------------------
// HX711 is read ONE conversion at a time whenever DOUT says data is ready.
// This avoids get_units(5) blocking for multiple conversions.

#define WEIGHT_MEDIAN_WINDOW 3
#define WEIGHT_STABILITY_WINDOW 5

// EMA: larger = faster response, smaller = smoother.
// 0.35 is a good compromise for a temporarily unmounted / shaky setup.
const float WEIGHT_EMA_ALPHA = 0.35f;

// Treat tiny negative/positive values around zero as zero.
const float WEIGHT_ZERO_DEADBAND_G = 4.0f;

// Bottle must be at least this heavy to be considered present.
const float WEIGHT_MIN_BOTTLE_G = 40.0f;

// A stable platform means recent filtered values stay within this range.
// Increase to 15-20g temporarily if your unmounted load cell is very shaky.
const float WEIGHT_STABLE_RANGE_G = 12.0f;

// Bottle-lift detection. A bottle is considered lifted when weight becomes
// less than max(25g, 30% of initial bottle weight) for consecutive samples.
const float WEIGHT_LIFT_ABS_G = 25.0f;
const float WEIGHT_LIFT_RATIO = 0.30f;
const uint8_t LIFT_CONFIRM_SAMPLES = 2;
const uint8_t RETURN_CONFIRM_SAMPLES = 2;

// Reject physically absurd readings caused by wiring glitches.
const float WEIGHT_MAX_REASONABLE_G = 10000.0f;

// -----------------------------------------------------------------------
// DATA STRUCTURES
// -----------------------------------------------------------------------
enum AppState {
  S_IDLE,
  S_REMINDER,
  S_WATER
};

struct Med {
  char name[32];
  char color[12];
  char mealTiming[8];  // "BEFORE" or "AFTER"

  uint8_t comp;
  uint8_t hr, mn;
  uint16_t yr;
  uint8_t mo, dy;
  int days;
  int waterMl;
  bool active;
  bool takenToday;
};

// -----------------------------------------------------------------------
// GLOBALS
// -----------------------------------------------------------------------
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
RTC_DS3231 rtc;
HX711 scale;
Preferences prefs;

Med meds[MAX_MEDS];
AppState state = S_IDLE;
int activeIdx = -1;
bool oledOK = false;
bool rtcOK = false;
bool ntpTimeOK = false;

// Timers
unsigned long tDisplay = 0;
unsigned long tBuzzer = 0;
unsigned long tSnooze = 0;
unsigned long tSync = 0;
unsigned long tLiveSync = 0;
unsigned long tBtn = 0;

bool buzzing = false;
String lastDate = "";

// -----------------------------------------------------------------------
// WEIGHT SENSOR GLOBALS
// -----------------------------------------------------------------------
float rawWeight = 0.0f;
float medianWeight = 0.0f;
float filteredWeight = 0.0f;
float displayWeight = 0.0f;
float baseWeight = 0.0f;

bool weightInitialized = false;
unsigned long lastWeightGoodMs = 0;

float medianBuf[WEIGHT_MEDIAN_WINDOW] = {0};
uint8_t medianCount = 0;
uint8_t medianPos = 0;

float stabilityBuf[WEIGHT_STABILITY_WINDOW] = {0};
uint8_t stabilityCount = 0;
uint8_t stabilityPos = 0;

bool bottleLifted = false;
bool bottleWasLifted = false;
uint8_t liftConfirmCount = 0;
uint8_t returnConfirmCount = 0;

// -----------------------------------------------------------------------
// UTILITY
// -----------------------------------------------------------------------
float roundTo1(float v) {
  return roundf(v);
}

float medianOfSmallArray(const float* input, uint8_t n) {
  if (n == 0) return 0.0f;

  float a[WEIGHT_MEDIAN_WINDOW];
  for (uint8_t i = 0; i < n; i++) a[i] = input[i];

  for (uint8_t i = 1; i < n; i++) {
    float key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = key;
  }

  if (n & 1) return a[n / 2];
  return (a[n / 2 - 1] + a[n / 2]) * 0.5f;
}

void resetWeightFilter() {
  rawWeight = 0.0f;
  medianWeight = 0.0f;
  filteredWeight = 0.0f;
  displayWeight = 0.0f;
  weightInitialized = false;
  lastWeightGoodMs = 0;

  medianCount = 0;
  medianPos = 0;
  stabilityCount = 0;
  stabilityPos = 0;

  for (uint8_t i = 0; i < WEIGHT_MEDIAN_WINDOW; i++) medianBuf[i] = 0.0f;
  for (uint8_t i = 0; i < WEIGHT_STABILITY_WINDOW; i++) stabilityBuf[i] = 0.0f;
}

void startWaterTracking() {
  baseWeight = 0.0f;
  displayWeight = 0.0f;
  bottleLifted = false;
  bottleWasLifted = false;
  liftConfirmCount = 0;
  returnConfirmCount = 0;
  resetWeightFilter();
}

// Read exactly ONE ready HX711 conversion. No 5-sample blocking average.
void updateWeightSensor() {
  if (!scale.is_ready()) return;

  float r = scale.get_units(1);
  if (!isfinite(r)) return;
  if (fabsf(r) > WEIGHT_MAX_REASONABLE_G) return;

  if (fabsf(r) <= WEIGHT_ZERO_DEADBAND_G) r = 0.0f;
  if (r < 0.0f && r > -20.0f) r = 0.0f;

  rawWeight = r;
  lastWeightGoodMs = millis();

  // 1) Median filter removes short spikes without averaging many blocking reads.
  medianBuf[medianPos] = r;
  medianPos = (medianPos + 1) % WEIGHT_MEDIAN_WINDOW;
  if (medianCount < WEIGHT_MEDIAN_WINDOW) medianCount++;

  float compact[WEIGHT_MEDIAN_WINDOW];
  for (uint8_t i = 0; i < medianCount; i++) compact[i] = medianBuf[i];
  medianWeight = medianOfSmallArray(compact, medianCount);

  // 2) EMA smooths the median result while remaining responsive.
  if (!weightInitialized) {
    filteredWeight = medianWeight;
    weightInitialized = true;
  } else {
    filteredWeight += WEIGHT_EMA_ALPHA * (medianWeight - filteredWeight);
  }

  if (fabsf(filteredWeight) <= WEIGHT_ZERO_DEADBAND_G) filteredWeight = 0.0f;

  // 3) Stability history used to lock initial/final bottle weights.
  stabilityBuf[stabilityPos] = filteredWeight;
  stabilityPos = (stabilityPos + 1) % WEIGHT_STABILITY_WINDOW;
  if (stabilityCount < WEIGHT_STABILITY_WINDOW) stabilityCount++;
}

bool weightDataFresh() {
  return weightInitialized && (millis() - lastWeightGoodMs < 800UL);
}

bool weightStable() {
  if (!weightDataFresh()) return false;
  if (stabilityCount < WEIGHT_STABILITY_WINDOW) return false;

  float mn = stabilityBuf[0];
  float mx = stabilityBuf[0];

  for (uint8_t i = 1; i < WEIGHT_STABILITY_WINDOW; i++) {
    if (stabilityBuf[i] < mn) mn = stabilityBuf[i];
    if (stabilityBuf[i] > mx) mx = stabilityBuf[i];
  }

  return (mx - mn) <= WEIGHT_STABLE_RANGE_G;
}

void updateBottleLiftState() {
  if (baseWeight < WEIGHT_MIN_BOTTLE_G || !weightDataFresh()) return;

  float liftThreshold = baseWeight * WEIGHT_LIFT_RATIO;
  if (liftThreshold < WEIGHT_LIFT_ABS_G) liftThreshold = WEIGHT_LIFT_ABS_G;

  bool liftCandidate = (medianWeight < liftThreshold);

  if (!bottleLifted) {
    returnConfirmCount = 0;

    if (liftCandidate) {
      if (liftConfirmCount < 255) liftConfirmCount++;
      if (liftConfirmCount >= LIFT_CONFIRM_SAMPLES) {
        bottleLifted = true;
        bottleWasLifted = true;
        liftConfirmCount = 0;
        Serial.println("[WATER] Bottle lift confirmed.");
      }
    } else {
      liftConfirmCount = 0;
    }
  } else {
    liftConfirmCount = 0;

    if (!liftCandidate && filteredWeight > WEIGHT_MIN_BOTTLE_G) {
      if (returnConfirmCount < 255) returnConfirmCount++;
      if (returnConfirmCount >= RETURN_CONFIRM_SAMPLES) {
        bottleLifted = false;
        returnConfirmCount = 0;
        Serial.println("[WATER] Bottle return detected; waiting for stable lock...");
      }
    } else {
      returnConfirmCount = 0;
    }
  }
}

// -----------------------------------------------------------------------
// SERVO
// -----------------------------------------------------------------------
// NOTE: This function still sends servo pulses synchronously for ~300ms.
// It is acceptable for this project flow, but it is not strictly non-blocking.
void servoWrite(int pin, int deg) {
  if (pin == 23) {
    SPI.end();
    delay(5);
  }

  int us = map(constrain(deg, 0, 180), 0, 180, 500, 2400);
  for (int i = 0; i < 15; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(us);
    digitalWrite(pin, LOW);
    delay(18);
  }
}

void openComp(int c) {
  int p = (c == 1) ? PIN_SERVO1 : (c == 2) ? PIN_SERVO2 : PIN_SERVO3;
  servoWrite(p, 90);
}

void closeComp(int c) {
  int p = (c == 1) ? PIN_SERVO1 : (c == 2) ? PIN_SERVO2 : PIN_SERVO3;
  servoWrite(p, 0);
}

// -----------------------------------------------------------------------
// RGB
// -----------------------------------------------------------------------
void rgb(uint8_t r, uint8_t g, uint8_t b) {
  if (RGB_COMMON_ANODE) {
    digitalWrite(PIN_RGB_R, r ? LOW : HIGH);
    digitalWrite(PIN_RGB_G, g ? LOW : HIGH);
    digitalWrite(PIN_RGB_B, b ? LOW : HIGH);
  } else {
    digitalWrite(PIN_RGB_R, r ? HIGH : LOW);
    digitalWrite(PIN_RGB_G, g ? HIGH : LOW);
    digitalWrite(PIN_RGB_B, b ? HIGH : LOW);
  }
}

void rgbColor(const char* c) {
  String s = String(c);
  s.trim();
  s.toUpperCase();

  if (s == "RED") rgb(1, 0, 0);
  else if (s == "GREEN") rgb(0, 1, 0);
  else if (s == "BLUE") rgb(0, 0, 1);
  else if (s == "YELLOW") rgb(1, 1, 0);
  else if (s == "CYAN") rgb(0, 1, 1);
  else if (s == "MAGENTA") rgb(1, 0, 1);
  else if (s == "ORANGE") rgb(1, 1, 0);
  else if (s == "PURPLE") rgb(1, 0, 1);
  else if (s == "PINK") rgb(1, 0, 1);
  else if (s == "WHITE") rgb(1, 1, 1);
  else rgb(0, 0, 0);
}

void rgbOff() {
  rgb(0, 0, 0);
}

// -----------------------------------------------------------------------
// BEFORE / AFTER MEAL LEDS
// -----------------------------------------------------------------------
void timingLEDsOff() {
  digitalWrite(PIN_LED_BEFORE, LOW);
  digitalWrite(PIN_LED_AFTER, LOW);
}

void showMealTimingLED(const Med& m) {
  String s = String(m.mealTiming);
  s.trim();
  s.toUpperCase();

  digitalWrite(PIN_LED_BEFORE, (s == "BEFORE") ? HIGH : LOW);
  digitalWrite(PIN_LED_AFTER, (s == "AFTER") ? HIGH : LOW);
}

// -----------------------------------------------------------------------
// TIME - Bangladesh real time (UTC+6) from NTP, with DS3231 fallback
// -----------------------------------------------------------------------
bool validDateTime(const DateTime& d) {
  return d.year() >= 2025 && d.year() <= 2099;
}

bool rtcHasValidTime() {
  if (!rtcOK) return false;
  // OSF/lostPower means the RTC may have stopped while VCC/VBAT was absent.
  if (rtc.lostPower()) return false;
  return validDateTime(rtc.now());
}

DateTime nowLocal() {
  // PRIMARY CLOCK: DS3231.  NTP is used at boot to set/correct it.
  // Once a CR2032 backup battery is installed, this keeps real time through power loss.
  if (rtcHasValidTime()) {
    return rtc.now();
  }

  // FALLBACK: ESP32 system clock, if NTP has already synchronized.
  // configTime() below applies Bangladesh UTC+6; do NOT add another +6 hours.
  struct tm ti;
  if (getLocalTime(&ti, 5)) {
    int yr = ti.tm_year + 1900;
    if (yr >= 2025 && yr <= 2099) {
      return DateTime(yr, ti.tm_mon + 1, ti.tm_mday,
                      ti.tm_hour, ti.tm_min, ti.tm_sec);
    }
  }

  // No trustworthy clock yet.
  return DateTime(2000, 1, 1, 0, 0, 0);
}

bool syncRealTimeFromNTP(uint32_t timeoutMs = 10000UL) {
  if (!WiFi.isConnected()) return false;

  // Bangladesh Standard Time = UTC + 6 hours, no daylight saving time.
  configTime(6 * 3600, 0,
             "pool.ntp.org",
             "time.nist.gov",
             "time.google.com");

  struct tm ti;
  unsigned long started = millis();
  while (millis() - started < timeoutMs) {
    if (getLocalTime(&ti, 250)) {
      int yr = ti.tm_year + 1900;
      if (yr >= 2025 && yr <= 2099) {
        DateTime realNow(yr, ti.tm_mon + 1, ti.tm_mday,
                         ti.tm_hour, ti.tm_min, ti.tm_sec);

        // Copy NTP time into DS3231 so it can be used if WiFi drops later.
        if (rtcOK) {
          rtc.adjust(realNow);
        }

        ntpTimeOK = true;
        Serial.printf("[TIME] NTP synced: %04d-%02d-%02d %02d:%02d:%02d (Bangladesh UTC+6)\n",
                      realNow.year(), realNow.month(), realNow.day(),
                      realNow.hour(), realNow.minute(), realNow.second());
        return true;
      }
    }
    delay(50);
  }

  ntpTimeOK = false;
  Serial.println("[TIME] NTP sync failed/timeout.");
  return false;
}

String fmtDate(const DateTime& d) {
  char b[12];
  snprintf(b, sizeof(b), "%04d-%02d-%02d", d.year(), d.month(), d.day());
  return String(b);
}

String fmtTime(const DateTime& d) {
  char b[10];
  snprintf(b, sizeof(b), "%02d:%02d:%02d", d.hour(), d.minute(), d.second());
  return String(b);
}

// -----------------------------------------------------------------------
// OLED SCREENS
// -----------------------------------------------------------------------
void screenIdle() {
  if (!oledOK) return;
  DateTime d = nowLocal();

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(32, 0);
  oled.print("HydraMedi");
  oled.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // 12-hour local time, e.g. 09:31 PM
  int hour12 = d.hour() % 12;
  if (hour12 == 0) hour12 = 12;
  const char* ampm = (d.hour() >= 12) ? "PM" : "AM";

  oled.setTextSize(2);
  oled.setCursor(4, 14);
  oled.printf("%02d:%02d", hour12, d.minute());
  oled.setTextSize(1);
  oled.setCursor(94, 20);
  oled.print(ampm);

  oled.setTextSize(1);
  oled.setCursor(20, 34);
  oled.printf("%04d-%02d-%02d", d.year(), d.month(), d.day());

  oled.drawLine(0, 46, 128, 46, SSD1306_WHITE);
  oled.setCursor(0, 52);
  oled.print(WiFi.isConnected() ? "WiFi:OK  USB Pwr" : "WiFi:-- USB Pwr");
  oled.display();
}

void screenMed(const Med& m) {
  if (!oledOK) return;

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(8, 0);
  oled.print("!! MEDICINE TIME !!");
  oled.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  oled.setCursor(0, 13);
  oled.printf("Med: %s", m.name);
  oled.setCursor(0, 23);
  oled.printf("Time:%02d:%02d  Box:#%d", m.hr, m.mn, m.comp);
  oled.setCursor(0, 33);
  oled.printf("Water:%dml", m.waterMl);

  oled.drawRect(0, 42, 128, 13, SSD1306_WHITE);
  oled.setCursor(3, 45);
  oled.printf("%s MEAL | %s", m.mealTiming, m.color);

  oled.setCursor(0, 58);
  oled.print(">> Press Green Button");
  oled.display();
}

void screenWater(float beforeG, float currentG, float consumedG, int needMl, bool lifted, bool waitingBottle) {
  if (!oledOK) return;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(18, 0);
  oled.print("** DRINK WATER **");
  oled.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  if (waitingBottle) {
    oled.setCursor(10, 20);
    oled.print("Place bottle on");
    oled.setCursor(26, 32);
    oled.print("the scale");
    oled.setCursor(5, 50);
    oled.print("Waiting for stable...");
  } else if (lifted) {
    oled.setCursor(15, 20);
    oled.print("Bottle Lifted...");
    oled.setCursor(10, 34);
    oled.print("Take your sips!");
    oled.setCursor(5, 50);
    oled.print(">> Put Bottle Back <<");
  } else {
    oled.setCursor(0, 13);
    oled.printf("Before : %5.0f g", beforeG);
    oled.setCursor(0, 22);
    oled.printf("Now    : %5.0f g", currentG);
    oled.setCursor(0, 31);
    oled.printf("Drank  : %5.0f ml/%d", consumedG, needMl);

    int pct = 0;
    if (needMl > 0) pct = constrain((int)(consumedG * 100.0f / needMl), 0, 100);

    oled.drawRect(0, 42, 128, 11, SSD1306_WHITE);
    oled.fillRect(2, 44, map(pct, 0, 100, 0, 124), 7, SSD1306_WHITE);
    oled.setCursor(48, 55);
    oled.printf("%d%%", pct);
  }

  oled.display();
}

void screenMsg(const char* msg) {
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(18, 20);
  oled.print("================");
  oled.setCursor(5, 32);
  oled.print(msg);
  oled.setCursor(18, 44);
  oled.print("================");
  oled.display();
}

// -----------------------------------------------------------------------
// FIREBASE
// -----------------------------------------------------------------------
void fbPost(const String& path, const String& json) {
  if (!WiFi.isConnected()) return;

  WiFiClientSecure cli;
  cli.setInsecure();
  HTTPClient http;
  http.begin(cli, String(FIREBASE_URL) + "/" + path + ".json");
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(json);
  Serial.printf("[FB] POST %s -> HTTP %d\n", path.c_str(), code);
  http.end();
}

void logEvent(const char* event, const char* med, float wml) {
  DateTime d = nowLocal();
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"event\":\"%s\",\"medicine\":\"%s\",\"waterMl\":%.1f,\"date\":\"%s\",\"time\":\"%s\"}",
           event, med, wml, fmtDate(d).c_str(), fmtTime(d).c_str());
  fbPost("logs/events", buf);
}

// -----------------------------------------------------------------------
// SIMPLE JSON HELPERS
// -----------------------------------------------------------------------
static String jsonStr(const String& src, const char* key) {
  String k = String("\"") + key + "\":\"";
  int i = src.indexOf(k);
  if (i < 0) return "";
  i += k.length();
  int j = src.indexOf('"', i);
  if (j < 0) return "";
  return src.substring(i, j);
}

static int jsonInt(const String& src, const char* key, int def = 0) {
  String k = String("\"") + key + "\":";
  int i = src.indexOf(k);
  if (i < 0) return def;
  return src.substring(i + k.length()).toInt();
}

static bool jsonBool(const String& src, const char* key, bool def = true) {
  String k = String("\"") + key + "\":";
  int i = src.indexOf(k);
  if (i < 0) return def;
  String v = src.substring(i + k.length(), i + k.length() + 5);
  return !v.startsWith("false");
}

void copyText(char* dst, size_t dstSize, const String& src) {
  if (dstSize == 0) return;
  strncpy(dst, src.c_str(), dstSize - 1);
  dst[dstSize - 1] = '\0';
}

void parseOneMed(const String& obj, int idx) {
  if (idx < 0 || idx >= MAX_MEDS) return;

  String n = jsonStr(obj, "name");
  if (n.length() == 0) n = "Med " + String(idx + 1);
  copyText(meds[idx].name, sizeof(meds[idx].name), n);

  String c = jsonStr(obj, "color");
  if (c.length() == 0) c = "WHITE";
  c.trim();
  c.toUpperCase();
  copyText(meds[idx].color, sizeof(meds[idx].color), c);

  // Firebase field expected: mealTiming = "BEFORE" or "AFTER"
  // Also accepts timing / mealRelation for convenience.
  String timing = jsonStr(obj, "mealTiming");
  if (timing.length() == 0) timing = jsonStr(obj, "timing");
  if (timing.length() == 0) timing = jsonStr(obj, "mealRelation");
  timing.trim();
  timing.toUpperCase();
  if (timing != "BEFORE" && timing != "AFTER") timing = "BEFORE";
  copyText(meds[idx].mealTiming, sizeof(meds[idx].mealTiming), timing);

  meds[idx].comp = constrain(jsonInt(obj, "compartment", idx + 1), 1, 3);
  meds[idx].hr = constrain(jsonInt(obj, "reminderHour", 8), 0, 23);
  meds[idx].mn = constrain(jsonInt(obj, "reminderMin", 0), 0, 59);
  meds[idx].yr = jsonInt(obj, "startYear", 2026);
  meds[idx].mo = constrain(jsonInt(obj, "startMonth", 8), 1, 12);
  meds[idx].dy = constrain(jsonInt(obj, "startDate", 6), 1, 31);
  meds[idx].days = max(1, jsonInt(obj, "durationDays", 7));
  meds[idx].waterMl = max(1, jsonInt(obj, "requiredWaterMl", 250));
  meds[idx].active = jsonBool(obj, "active", true);

  Serial.printf("[PARSED] Med[%d] %s @ %02d:%02d Comp%d %s MEAL\n",
                idx, meds[idx].name, meds[idx].hr, meds[idx].mn,
                meds[idx].comp, meds[idx].mealTiming);
}

void syncFB() {
  if (!WiFi.isConnected()) return;

  WiFiClientSecure cli;
  cli.setInsecure();
  HTTPClient http;
  http.begin(cli, String(FIREBASE_URL) + "/schedules.json");

  int code = http.GET();
  if (code == 200) {
    String body = http.getString();
    Serial.println("[FB] Syncing Schedules...");

    for (int i = 1; i <= MAX_MEDS; i++) {
      int pos = body.indexOf("\"id\":" + String(i));
      if (pos < 0) pos = body.indexOf("\"compartment\":" + String(i));
      if (pos < 0) pos = body.indexOf(String("\"") + String(i) + "\":{");

      if (pos >= 0) {
        int s = body.lastIndexOf('{', pos);
        int e = body.indexOf('}', pos);
        if (s >= 0 && e >= 0) parseOneMed(body.substring(s, e + 1), i - 1);
      }
    }

    prefs.begin("hm", false);
    prefs.putBytes("meds", meds, sizeof(meds));
    prefs.end();
    Serial.println("[FB] Sync OK - Schedules Updated.");
  } else {
    Serial.printf("[FB] Schedule GET failed: HTTP %d\n", code);
  }

  http.end();
}

void syncLiveState() {
  if (!WiFi.isConnected()) return;

  char buf[320];
  const char* sStr = (state == S_IDLE) ? "IDLE" :
                     (state == S_REMINDER) ? "REMINDER" : "WATER";

  bool lifted = bottleLifted;
  float drunk = 0.0f;

  if (baseWeight > WEIGHT_MIN_BOTTLE_G && displayWeight > WEIGHT_MIN_BOTTLE_G) {
    drunk = baseWeight - displayWeight;
    if (drunk < 0.0f) drunk = 0.0f;
  }

  snprintf(buf, sizeof(buf),
           "{\"state\":\"%s\",\"medIdx\":%d,\"sensorG\":%.1f,\"currentG\":%.1f,\"beforeG\":%.1f,\"drank\":%.1f,\"lifted\":%s,\"stable\":%s}",
           sStr, activeIdx, filteredWeight, displayWeight, baseWeight, drunk,
           lifted ? "true" : "false",
           weightStable() ? "true" : "false");

  WiFiClientSecure cli;
  cli.setInsecure();
  HTTPClient http;
  http.begin(cli, String(FIREBASE_URL) + "/LiveState.json");
  http.addHeader("Content-Type", "application/json");
  http.PUT(buf);
  http.end();
}

// -----------------------------------------------------------------------
// DEFAULT SCHEDULE
// -----------------------------------------------------------------------
void defaultSchedule() {
  DateTime d = nowLocal();

  for (int i = 0; i < MAX_MEDS; i++) {
    int offset = (i == 0) ? 1 : (i == 1) ? 5 : 10;
    int tm = d.hour() * 60 + d.minute() + offset;

    meds[i].hr = (tm / 60) % 24;
    meds[i].mn = tm % 60;
    meds[i].yr = d.year();
    meds[i].mo = d.month();
    meds[i].dy = d.day();
    meds[i].days = 7;
    meds[i].waterMl = 250;
    meds[i].active = true;
    meds[i].takenToday = false;
    meds[i].comp = i + 1;
  }

  copyText(meds[0].name, sizeof(meds[0].name), "Paracetamol 500mg");
  copyText(meds[0].color, sizeof(meds[0].color), "RED");
  copyText(meds[0].mealTiming, sizeof(meds[0].mealTiming), "AFTER");

  copyText(meds[1].name, sizeof(meds[1].name), "Vitamin C 1000mg");
  copyText(meds[1].color, sizeof(meds[1].color), "GREEN");
  copyText(meds[1].mealTiming, sizeof(meds[1].mealTiming), "AFTER");
  meds[1].waterMl = 200;
  meds[1].days = 30;

  copyText(meds[2].name, sizeof(meds[2].name), "Omeprazole 20mg");
  copyText(meds[2].color, sizeof(meds[2].color), "BLUE");
  copyText(meds[2].mealTiming, sizeof(meds[2].mealTiming), "BEFORE");
  meds[2].waterMl = 300;
  meds[2].days = 14;

  Serial.printf("[SCHED] Default: Med[0] -> %02d:%02d\n", meds[0].hr, meds[0].mn);
}

// -----------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n=== HydraMedi v4.3 Starting ===");

  // 1. GPIO
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_SERVO1, OUTPUT);
  pinMode(PIN_SERVO2, OUTPUT);
  pinMode(PIN_SERVO3, OUTPUT);
  pinMode(PIN_HX_CLK, OUTPUT);

  pinMode(PIN_LED_BEFORE, OUTPUT);
  pinMode(PIN_LED_AFTER, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_SERVO1, LOW);
  digitalWrite(PIN_SERVO2, LOW);
  digitalWrite(PIN_SERVO3, LOW);
  digitalWrite(PIN_HX_CLK, LOW);
  timingLEDsOff();
  rgbOff();

  // RGB startup test
  Serial.printf("[RGB] Type: %s\n", RGB_COMMON_ANODE ? "COMMON ANODE" : "COMMON CATHODE");
  rgb(1, 0, 0); delay(300);
  rgb(0, 1, 0); delay(300);
  rgb(0, 0, 1); delay(300);
  rgbOff();

  // Timing LED startup test
  digitalWrite(PIN_LED_BEFORE, HIGH); delay(250);
  digitalWrite(PIN_LED_BEFORE, LOW);
  digitalWrite(PIN_LED_AFTER, HIGH); delay(250);
  digitalWrite(PIN_LED_AFTER, LOW);

  // 2. I2C + OLED
  Wire.begin(PIN_SDA, PIN_SCL);
  delay(50);

  if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    oledOK = true;
  } else if (oled.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    oledOK = true;
  }

  if (oledOK) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(25, 20);
    oled.print("HydraMedi v4.3");
    oled.setCursor(20, 36);
    oled.print("Initializing...");
    oled.display();
  } else {
    Serial.println("[WARN] OLED not found!");
  }

  // 3. RTC
  rtcOK = rtc.begin();
  if (rtcOK) {
    Serial.println("[OK] DS3231 found.");
    if (rtc.lostPower()) {
      Serial.println("[RTC] Clock lost power / has no valid retained time.");
      Serial.println("[RTC] NTP will set it if WiFi is available.");
    } else if (validDateTime(rtc.now())) {
      Serial.printf("[RTC] Retained time: %s %s\n",
                    fmtDate(rtc.now()).c_str(), fmtTime(rtc.now()).c_str());
    } else {
      Serial.println("[RTC] Stored date/time is invalid; NTP correction required.");
    }
  } else {
    Serial.println("[WARN] DS3231 not found.");
  }

  // 4. HX711
  Serial.println("[INIT] HX711 begin...");
  scale.begin(PIN_HX_DAT, PIN_HX_CLK);
  scale.set_scale(HX711_CALIBRATION);

  if (scale.wait_ready_timeout(1200)) {
    Serial.println("[OK] HX711 ready. Taring...");
    scale.tare(10);
    resetWeightFilter();
    Serial.println("[OK] HX711 tared.");
  } else {
    Serial.println("[WARN] HX711 not ready (check wiring/power). Project continues.");
  }

  // 5. WiFi
  Serial.printf("[INIT] Connecting WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long wt = millis();
  while (!WiFi.isConnected() && millis() - wt < 8000UL) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.isConnected()) {
    Serial.printf("\n[OK] WiFi connected: %s\n", WiFi.localIP().toString().c_str());

    // Get real Bangladesh time from internet and sync the DS3231.
    // Do not continue using 00:00/1970 if NTP has not actually synchronized.
    if (!syncRealTimeFromNTP(10000UL)) {
      if (rtcHasValidTime()) {
        Serial.printf("[TIME] Using DS3231 fallback: %s %s\n",
                      fmtDate(rtc.now()).c_str(), fmtTime(rtc.now()).c_str());
      } else {
        Serial.println("[TIME] ERROR: No valid real time available yet.");
      }
    }
  } else {
    Serial.println("\n[WARN] WiFi timeout. Running offline.");
  }

  // 6. Schedule
  prefs.begin("hm", false);
  prefs.clear();
  prefs.end();

  // Start with safe defaults so Firebase parse failure never leaves zeroed meds.
  defaultSchedule();

  if (WiFi.isConnected()) {
    syncFB();
  }

  Serial.printf("[OK] Setup complete! Local time: %s\n", fmtTime(nowLocal()).c_str());
  for (int i = 0; i < MAX_MEDS; i++) {
    Serial.printf("  Med[%d] %-20s -> %02d:%02d active=%d %s MEAL\n",
                  i, meds[i].name, meds[i].hr, meds[i].mn,
                  meds[i].active, meds[i].mealTiming);
  }

  timingLEDsOff();
  screenIdle();
}

// -----------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------
void loop() {
  unsigned long ms = millis();
  DateTime d = nowLocal();

  // Always service HX711 whenever a conversion is ready.
  // One conversion only; no 5-sample blocking call.
  updateWeightSensor();

  // Daily reset
  String today = fmtDate(d);
  if (today != lastDate && lastDate != "") {
    for (int i = 0; i < MAX_MEDS; i++) meds[i].takenToday = false;
    Serial.println("[RESET] Daily takenToday flags cleared.");
  }
  lastDate = today;

  // Button
  bool btn = false;
  if (digitalRead(PIN_BUTTON) == LOW && ms - tBtn > DEBOUNCE_MS) {
    btn = true;
    tBtn = ms;
    Serial.println("[BTN] Green button pressed!");
  }

  // Buzzer auto-off
  if (buzzing && ms - tBuzzer >= BUZZER_DURATION_MS) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzing = false;
    Serial.println("[BUZ] Auto-off after 20s.");
  }

  // Live Firebase push
  unsigned long liveInterval = (state == S_IDLE) ? 10000UL : 2000UL;
  if (ms - tLiveSync >= liveInterval) {
    tLiveSync = ms;
    syncLiveState();
  }

  switch (state) {

    // -------------------------------------------------------------------
    case S_IDLE: {
      timingLEDsOff();

      if (ms - tDisplay >= DISPLAY_REFRESH_MS) {
        tDisplay = ms;
        screenIdle();
        Serial.printf("[CLK] %02d:%02d:%02d\n", d.hour(), d.minute(), d.second());
      }

      if (ms - tSync >= FIREBASE_SYNC_MS) {
        tSync = ms;
        syncFB();
      }

      // Alarm check
      for (int i = 0; i < MAX_MEDS; i++) {
        Med& m = meds[i];

        if (m.active && !m.takenToday &&
            d.hour() == m.hr && d.minute() == m.mn) {

          Serial.printf("[ALARM] %s at %02d:%02d | %s MEAL | color=%s\n",
                        m.name, m.hr, m.mn, m.mealTiming, m.color);

          activeIdx = i;
          rgbColor(m.color);
          showMealTimingLED(m);
          openComp(m.comp);

          digitalWrite(PIN_BUZZER, HIGH);
          buzzing = true;
          tBuzzer = ms;
          tSnooze = ms;
          tDisplay = 0;

          screenMed(m);
          state = S_REMINDER;
          break;
        }
      }
      break;
    }

    // -------------------------------------------------------------------
    case S_REMINDER: {
      if (activeIdx < 0) {
        timingLEDsOff();
        rgbOff();
        state = S_IDLE;
        break;
      }

      Med& m = meds[activeIdx];

      // Only refresh OLED at a controlled interval.
      if (ms - tDisplay >= REMINDER_DISPLAY_MS) {
        tDisplay = ms;
        screenMed(m);
        rgbColor(m.color);
        showMealTimingLED(m);
      }

      // 30-min snooze repeat
      if (ms - tSnooze >= SNOOZE_INTERVAL_MS) {
        tSnooze = ms;
        tBuzzer = ms;
        digitalWrite(PIN_BUZZER, HIGH);
        buzzing = true;
        Serial.println("[SNOOZE] 30-min repeat alarm.");
      }

      // Confirm medicine taken
      if (btn) {
        m.takenToday = true;

        digitalWrite(PIN_BUZZER, LOW);
        buzzing = false;
        rgbOff();
        timingLEDsOff();
        closeComp(m.comp);

        logEvent("Medicine Taken", m.name, 0.0f);

        prefs.begin("hm", false);
        prefs.putBytes("meds", meds, sizeof(meds));
        prefs.end();

        startWaterTracking();
        screenMsg("Medicine Taken!");
        delay(700);  // short UX pause; weight filter restarts after this

        state = S_WATER;
        tSnooze = millis();
        tDisplay = 0;
      }
      break;
    }

    // -------------------------------------------------------------------
    case S_WATER: {
      if (activeIdx < 0) {
        state = S_IDLE;
        break;
      }

      Med& m = meds[activeIdx];

      // STEP 1: Lock initial bottle weight only when sensor is actually stable.
      if (baseWeight < WEIGHT_MIN_BOTTLE_G) {
        if (weightStable() && filteredWeight > WEIGHT_MIN_BOTTLE_G) {
          baseWeight = roundTo1(filteredWeight);
          displayWeight = baseWeight;
          bottleLifted = false;
          bottleWasLifted = false;
          liftConfirmCount = 0;
          returnConfirmCount = 0;

          Serial.printf("[WATER] Initial stable bottle locked: %.1fg\n", baseWeight);
        }

        if (ms - tDisplay >= WATER_DISPLAY_MS) {
          tDisplay = ms;
          screenWater(0.0f, filteredWeight, 0.0f, m.waterMl, false, true);
        }

        // Allow user to skip even if no stable bottle was detected.
        if (btn) {
          logEvent("Water Skipped", m.name, 0.0f);
          screenMsg("Water Skipped");
          digitalWrite(PIN_BUZZER, LOW);
          buzzing = false;
          state = S_IDLE;
          activeIdx = -1;
          tDisplay = 0;
        }
        break;
      }

      // STEP 2: Detect bottle lift / return using consecutive confirmations.
      updateBottleLiftState();

      // STEP 3: After a confirmed lift and return, lock ONE stable final weight.
      if (!bottleLifted && bottleWasLifted &&
          weightStable() && filteredWeight > WEIGHT_MIN_BOTTLE_G) {

        float newFinal = roundTo1(filteredWeight);

        // Prevent display from jumping upward due to tiny drift/noise.
        // A real refill can still be shown, but consumed water will clamp to zero.
        displayWeight = newFinal;
        bottleWasLifted = false;

        Serial.printf("[WATER] Stable final bottle locked: %.1fg\n", displayWeight);
      }

      // STEP 4: Initial grams - final grams ~= ml of water consumed.
      float drank = 0.0f;
      if (displayWeight > WEIGHT_MIN_BOTTLE_G) {
        drank = baseWeight - displayWeight;
        if (drank < 0.0f) drank = 0.0f;
      }

      if (ms - tDisplay >= WATER_DISPLAY_MS) {
        tDisplay = ms;
        screenWater(baseWeight,
                    bottleLifted ? 0.0f : displayWeight,
                    drank,
                    m.waterMl,
                    bottleLifted,
                    false);
      }

      // Complete only after bottle is back and a stable final reading is locked.
      if (!bottleLifted && !bottleWasLifted &&
          displayWeight > WEIGHT_MIN_BOTTLE_G &&
          drank >= (float)m.waterMl) {

        Serial.printf("[WATER] Target reached: %.0fml / %dml\n", drank, m.waterMl);
        logEvent("Water Drank", m.name, drank);
        screenMsg("Water Complete!");

        digitalWrite(PIN_BUZZER, LOW);
        buzzing = false;
        rgbOff();
        timingLEDsOff();
        delay(900);

        state = S_IDLE;
        activeIdx = -1;
        tDisplay = 0;
        break;
      }

      // Water reminder every 30 min
      if (ms - tSnooze >= SNOOZE_INTERVAL_MS) {
        tSnooze = ms;
        tBuzzer = ms;
        digitalWrite(PIN_BUZZER, HIGH);
        buzzing = true;
        Serial.println("[WATER] 30-min drink reminder.");
      }

      // Green button = log partial / skip
      if (btn) {
        logEvent("Water Logged", m.name, drank);
        screenMsg("Water Logged!");

        digitalWrite(PIN_BUZZER, LOW);
        buzzing = false;
        rgbOff();
        timingLEDsOff();
        delay(700);

        state = S_IDLE;
        activeIdx = -1;
        tDisplay = 0;
      }

      break;
    }
  }
}