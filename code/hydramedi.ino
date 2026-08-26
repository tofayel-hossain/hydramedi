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

// Timing LEDs for 3 compartments (Yellow = BEFORE meal, Green = AFTER meal)
#define PIN_LED_C1_BEFORE 32   // Compartment 1 (Morning)   - Before Meal
#define PIN_LED_C1_AFTER  33   // Compartment 1 (Morning)   - After Meal
#define PIN_LED_C2_BEFORE 12   // Compartment 2 (Afternoon) - Before Meal
#define PIN_LED_C2_AFTER  15   // Compartment 2 (Afternoon) - After Meal
#define PIN_LED_C3_BEFORE 2    // Compartment 3 (Night)     - Before Meal
#define PIN_LED_C3_AFTER  17   // Compartment 3 (Night)     - After Meal

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

enum FirebaseStatus {
  FB_NOT_CHECKED,
  FB_OK,
  FB_EMPTY,
  FB_ERROR
};

FirebaseStatus firebaseStatus = FB_NOT_CHECKED;
int firebaseActiveCount = 0;
String firebaseError = "";

// -----------------------------------------------------------------------
// ALARM DUPLICATE-PREVENTION
// -----------------------------------------------------------------------
// Firebase is still the ONLY source of the schedule. These keys are only
// used to remember that a Firebase alarm at a particular date/time has
// already been triggered. This prevents the same alarm from firing again
// after the medicine/water flow returns to S_IDLE and Firebase is synced.
String lastTriggeredKey[MAX_MEDS] = {"", "", ""};

String makeAlarmKey(int idx, const Med& m, const DateTime& d);

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

// Read exactly ONE ready HX711 conversion with zero-delay guard.
// If load cell is disconnected/unready, returns immediately without blocking loop.
void updateWeightSensor() {
  if (digitalRead(PIN_HX_DAT) != LOW) return;

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
  pinMode(pin, OUTPUT);

  if (pin == 23) {
    SPI.end();
    delay(5);
  }

  int us = map(constrain(deg, 0, 180), 0, 180, 500, 2400);
  Serial.printf("[SERVO] Motor Pin GPIO %d -> %d deg (%d us pulse)\n", pin, deg, us);

  // 40 pulses * 20ms = 800ms total window so servos reliably reach target angle
  for (int i = 0; i < 40; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(us);
    digitalWrite(pin, LOW);
    delay(18);
  }
}

void openComp(int c) {
  int p = (c == 1) ? PIN_SERVO1 : (c == 2) ? PIN_SERVO2 : PIN_SERVO3;
  Serial.printf("[SERVO] Opening Compartment %d (GPIO %d)\n", c, p);
  servoWrite(p, 90);
}

void closeComp(int c) {
  int p = (c == 1) ? PIN_SERVO1 : (c == 2) ? PIN_SERVO2 : PIN_SERVO3;
  Serial.printf("[SERVO] Closing Compartment %d (GPIO %d)\n", c, p);
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
// BEFORE / AFTER MEAL LEDS (6 LEDs: 2 per compartment)
// -----------------------------------------------------------------------
void timingLEDsOff() {
  digitalWrite(PIN_LED_C1_BEFORE, LOW);
  digitalWrite(PIN_LED_C1_AFTER,  LOW);
  digitalWrite(PIN_LED_C2_BEFORE, LOW);
  digitalWrite(PIN_LED_C2_AFTER,  LOW);
  digitalWrite(PIN_LED_C3_BEFORE, LOW);
  digitalWrite(PIN_LED_C3_AFTER,  LOW);
}

void showMealTimingLED(const Med& m) {
  timingLEDsOff();

  String s = String(m.mealTiming);
  s.trim();
  s.toUpperCase();
  bool isBefore = (s == "BEFORE");

  uint8_t c = (m.comp >= 1 && m.comp <= 3) ? m.comp : 1;

  Serial.printf("[LED] showMealTimingLED: Compartment %d | Timing %s | Before=%s\n",
                c, isBefore ? "BEFORE" : "AFTER", isBefore ? "YES" : "NO");

  if (c == 1) {
    digitalWrite(PIN_LED_C1_BEFORE, isBefore ? HIGH : LOW);
    digitalWrite(PIN_LED_C1_AFTER,  !isBefore ? HIGH : LOW);
  } else if (c == 2) {
    digitalWrite(PIN_LED_C2_BEFORE, isBefore ? HIGH : LOW);
    digitalWrite(PIN_LED_C2_AFTER,  !isBefore ? HIGH : LOW);
  } else if (c == 3) {
    digitalWrite(PIN_LED_C3_BEFORE, isBefore ? HIGH : LOW);
    digitalWrite(PIN_LED_C3_AFTER,  !isBefore ? HIGH : LOW);
  }
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

String makeAlarmKey(int idx, const Med& m, const DateTime& d) {
  // Include medicine identity + date + exact Firebase time.
  // If Firebase changes the schedule to another minute, it becomes a new event.
  return String(idx) + "|" + String(m.comp) + "|" +
         String(m.name) + "|" + fmtDate(d) + "|" +
         String(m.hr) + ":" + String(m.mn);
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

  int hour12 = d.hour() % 12;
  if (hour12 == 0) hour12 = 12;
  const char* ampm = (d.hour() >= 12) ? "PM" : "AM";

  oled.setTextSize(2);
  oled.setCursor(4, 14);
  oled.printf("%02d:%02d", hour12, d.minute());
  oled.setTextSize(1);
  oled.setCursor(94, 20);
  oled.print(ampm);

  oled.setCursor(20, 34);
  oled.printf("%04d-%02d-%02d", d.year(), d.month(), d.day());

  oled.drawLine(0, 46, 128, 46, SSD1306_WHITE);
  oled.setCursor(0, 52);

  if (firebaseStatus == FB_ERROR) {
    oled.print("FB: ERROR - CHECK DATA");
  } else if (firebaseStatus == FB_EMPTY) {
    oled.print("FB: NO SCHEDULE");
  } else if (firebaseStatus == FB_OK) {
    oled.printf("FB: OK  %d SCHEDULE%s", firebaseActiveCount,
                firebaseActiveCount == 1 ? "" : "S");
  } else {
    oled.print("FB: WAITING...");
  }

  oled.display();
}

void screenFirebaseError() {
  if (!oledOK) return;

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(17, 0);
  oled.print("FIREBASE ERROR");
  oled.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  oled.setCursor(0, 18);
  oled.print("Schedule read failed.");
  oled.setCursor(0, 29);
  oled.print("No new schedule used.");
  oled.setCursor(0, 40);
  oled.print("Check WiFi/Firebase.");
  oled.setCursor(0, 52);
  oled.print("System will retry.");
  oled.display();
}


// -----------------------------------------------------------------------
// STARTUP / LOADING ANIMATION
// -----------------------------------------------------------------------
void startupText(const String& title, const String& sub, int progress) {
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(32, 0);
  oled.print("HydraMedi");
  oled.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  oled.setTextSize(2);
  oled.setCursor(8, 18);
  oled.print(title);
  oled.setTextSize(1);
  oled.setCursor(8, 40);
  oled.print(sub);
  oled.drawRect(8, 52, 112, 8, SSD1306_WHITE);
  int w = constrain(progress, 0, 100) * 108 / 100;
  if (w > 0) oled.fillRect(10, 54, w, 4, SSD1306_WHITE);
  oled.display();
}

void startupAnimation() {
  Serial.println("[STARTUP] HydraMedi loading animation...");

  // Text is revealed serially on OLED.
  const char* name = "HydraMedi";
  for (int i = 1; i <= 9; i++) {
    if (oledOK) {
      oled.clearDisplay();
      oled.setTextColor(SSD1306_WHITE);
      oled.setTextSize(2);
      oled.setCursor(12, 18);
      for (int j = 0; j < i; j++) oled.print(name[j]);
      oled.setTextSize(1);
      oled.setCursor(34, 40);
      oled.print("Loading...");
      oled.display();
    }
    Serial.print(name[i-1]);
    delay(120);
  }
  Serial.println();

  // RGB self-test: RED -> GREEN -> BLUE -> OFF.
  const int rgbMs = 180;
  Serial.println("[STARTUP] RGB: RED -> GREEN -> BLUE");
  rgb(1, 0, 0); startupText("RGB", "RED", 25); delay(rgbMs);
  rgb(0, 1, 0); startupText("RGB", "GREEN", 50); delay(rgbMs);
  rgb(0, 0, 1); startupText("RGB", "BLUE", 75); delay(rgbMs);
  rgbOff();

  // Six meal-timing LEDs: each LED ON then OFF in sequence.
  struct LedStep { int pin; const char* label; };
  const LedStep steps[] = {
    {PIN_LED_C1_BEFORE, "C1 BEFORE"},
    {PIN_LED_C1_AFTER,  "C1 AFTER"},
    {PIN_LED_C2_BEFORE, "C2 BEFORE"},
    {PIN_LED_C2_AFTER,  "C2 AFTER"},
    {PIN_LED_C3_BEFORE, "C3 BEFORE"},
    {PIN_LED_C3_AFTER,  "C3 AFTER"}
  };

  Serial.println("[STARTUP] Meal LEDs: each LED ON/OFF sequentially");
  for (int i = 0; i < 6; i++) {
    int progress = 78 + (i + 1) * 3;
    Serial.printf("[STARTUP] %s ON\n", steps[i].label);
    digitalWrite(steps[i].pin, HIGH);
    startupText("LED TEST", steps[i].label, progress);
    delay(180);
    digitalWrite(steps[i].pin, LOW);
    delay(100);
  }
  timingLEDsOff();

  startupText("HydraMedi", "Hardware Ready", 100);
  Serial.println("[STARTUP] Animation complete.");
  delay(350);
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
// -----------------------------------------------------------------------
// FIREBASE URL HELPER
// -----------------------------------------------------------------------
String getFbUrl(const String& path) {
  String base = String(FIREBASE_URL);
  base.trim();
  while (base.endsWith("/")) {
    base.remove(base.length() - 1);
  }
  if (path.startsWith("/")) return base + path;
  return base + "/" + path;
}

void fbPost(const String& path, const String& json) {
  if (!WiFi.isConnected()) return;

  WiFiClientSecure cli;
  cli.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.begin(cli, getFbUrl(path + ".json"));
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
// ROBUST JSON HELPERS (Handles spaces around colons & quotes)
// -----------------------------------------------------------------------
static String getJsonRawValue(const String& src, const char* key) {
  String k = String("\"") + key + "\"";
  int keyPos = src.indexOf(k);
  if (keyPos < 0) return "";

  int colonPos = src.indexOf(':', keyPos + k.length());
  if (colonPos < 0) return "";

  int start = colonPos + 1;
  while (start < (int)src.length() && (src[start] == ' ' || src[start] == '\t' || src[start] == '\r' || src[start] == '\n')) {
    start++;
  }
  if (start >= (int)src.length()) return "";
  return src.substring(start);
}

static String jsonStr(const String& src, const char* key) {
  String val = getJsonRawValue(src, key);
  if (val.length() == 0) return "";

  if (val[0] == '"') {
    int endQuote = val.indexOf('"', 1);
    if (endQuote >= 0) return val.substring(1, endQuote);
  }
  return "";
}

static int jsonInt(const String& src, const char* key, int def = 0) {
  String val = getJsonRawValue(src, key);
  if (val.length() == 0) return def;

  if (val[0] == '"') val = val.substring(1);
  return val.toInt();
}

static bool jsonBool(const String& src, const char* key, bool def = true) {
  String val = getJsonRawValue(src, key);
  if (val.length() == 0) return def;
  val.trim();
  val.toLowerCase();
  if (val.startsWith("false") || val.startsWith("\"false\"") || val.startsWith("0")) return false;
  return true;
}

void copyText(char* dst, size_t dstSize, const String& src) {
  if (dstSize == 0) return;
  strncpy(dst, src.c_str(), dstSize - 1);
  dst[dstSize - 1] = '\0';
}

// Extract a complete JSON object {...} starting from searchPos in body
String extractJsonObject(const String& body, int bracePos) {
  if (bracePos < 0 || bracePos >= (int)body.length() || body[bracePos] != '{') return "";

  int depth = 0;
  int objEnd = -1;

  for (int i = bracePos; i < (int)body.length(); i++) {
    if (body[i] == '{') depth++;
    else if (body[i] == '}') {
      depth--;
      if (depth == 0) { objEnd = i; break; }
    }
  }
  if (objEnd < 0) return "";
  return body.substring(bracePos, objEnd + 1);
}

void clearAllSchedules() {
  for (int i = 0; i < MAX_MEDS; i++) {
    memset(&meds[i], 0, sizeof(Med));
    meds[i].comp = i + 1;
    meds[i].active = false;
    meds[i].takenToday = false;
  }
  firebaseActiveCount = 0;
}

bool hasJsonKey(const String& src, const char* key) {
  String k = String("\"") + key + "\"";
  return src.indexOf(k) >= 0;
}

int daysInMonth(int year, int month) {
  if (month < 1 || month > 12) return 0;
  static const uint8_t days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (month == 2) {
    bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
    return leap ? 29 : 28;
  }
  return days[month - 1];
}

bool validScheduleDate(int year, int month, int day) {
  return year >= 2025 && year <= 2099 &&
         month >= 1 && month <= 12 &&
         day >= 1 && day <= daysInMonth(year, month);
}

bool parseOneMed(const String& obj, int idx) {
  if (idx < 0 || idx >= MAX_MEDS) return false;

  // Required Firebase fields. Missing fields are an invalid schedule,
  // not an instruction to invent a default value.
  const char* required[] = {
    "name", "compartment", "color", "mealTiming",
    "reminderHour", "reminderMin",
    "startYear", "startMonth", "startDate",
    "durationDays", "requiredWaterMl", "active"
  };
  for (const char* key : required) {
    if (!hasJsonKey(obj, key)) {
      Serial.printf("[FB] Invalid schedule %d: missing field '%s'\n", idx + 1, key);
      return false;
    }
  }

  String n = jsonStr(obj, "name");
  String c = jsonStr(obj, "color");
  String timing = jsonStr(obj, "mealTiming");
  n.trim(); c.trim(); timing.trim();
  c.toUpperCase(); timing.toUpperCase();

  if (n.length() == 0 || c.length() == 0) {
    Serial.printf("[FB] Invalid schedule %d: name/color is empty.\n", idx + 1);
    return false;
  }

  if (timing != "BEFORE" && timing != "AFTER") {
    Serial.printf("[FB] Invalid schedule %d: mealTiming='%s'. Use BEFORE or AFTER.\n",
                  idx + 1, timing.c_str());
    return false;
  }

  int comp = jsonInt(obj, "compartment", -1);
  int h = jsonInt(obj, "reminderHour", -1);
  int m = jsonInt(obj, "reminderMin", -1);
  int year = jsonInt(obj, "startYear", -1);
  int month = jsonInt(obj, "startMonth", -1);
  int day = jsonInt(obj, "startDate", -1);
  int duration = jsonInt(obj, "durationDays", -1);
  int water = jsonInt(obj, "requiredWaterMl", -1);
  bool active = jsonBool(obj, "active", false);

  if (comp != idx + 1) {
    Serial.printf("[FB] Invalid schedule %d: compartment=%d but expected %d.\n",
                  idx + 1, comp, idx + 1);
    return false;
  }

  if (h < 0 || h > 23 || m < 0 || m > 59) {
    Serial.printf("[FB] Invalid schedule %d: bad time %02d:%02d.\n", idx + 1, h, m);
    return false;
  }

  if (!validScheduleDate(year, month, day)) {
    Serial.printf("[FB] Invalid schedule %d: bad start date %04d-%02d-%02d.\n",
                  idx + 1, year, month, day);
    return false;
  }

  if (duration <= 0 || duration > 3650) {
    Serial.printf("[FB] Invalid schedule %d: durationDays=%d.\n", idx + 1, duration);
    return false;
  }

  if (water <= 0 || water > 10000) {
    Serial.printf("[FB] Invalid schedule %d: requiredWaterMl=%d.\n", idx + 1, water);
    return false;
  }

  memset(&meds[idx], 0, sizeof(Med));
  copyText(meds[idx].name, sizeof(meds[idx].name), n);
  copyText(meds[idx].color, sizeof(meds[idx].color), c);
  copyText(meds[idx].mealTiming, sizeof(meds[idx].mealTiming), timing);

  meds[idx].comp = comp;
  meds[idx].hr = (uint8_t)h;
  meds[idx].mn = (uint8_t)m;
  meds[idx].yr = (uint16_t)year;
  meds[idx].mo = (uint8_t)month;
  meds[idx].dy = (uint8_t)day;
  meds[idx].days = duration;
  meds[idx].waterMl = water;
  meds[idx].active = active;
  meds[idx].takenToday = false;

  Serial.printf("[PARSED] Med[%d] %s @ %02d:%02d Comp%d %s MEAL | %04d-%02d-%02d for %d days | active=%d\n",
                idx, meds[idx].name, meds[idx].hr, meds[idx].mn,
                meds[idx].comp, meds[idx].mealTiming,
                meds[idx].yr, meds[idx].mo, meds[idx].dy, meds[idx].days,
                meds[idx].active);
  return true;
}

bool scheduleIsValidToday(const Med& m, const DateTime& d) {
  if (!m.active) return false;
  if (m.days <= 0) return false;

  DateTime start(m.yr, m.mo, m.dy, 0, 0, 0);
  DateTime today(d.year(), d.month(), d.day(), 0, 0, 0);
  if (today < start) return false;

  TimeSpan elapsed = today - start;
  return elapsed.days() < m.days;
}

bool fetchFirebaseSchedules() {
  if (!WiFi.isConnected()) {
    firebaseStatus = FB_ERROR;
    firebaseError = "WiFi disconnected";
    return false;
  }

  Serial.println("[FB] Syncing schedules from Firebase...");

  Med oldMeds[MAX_MEDS];
  memcpy(oldMeds, meds, sizeof(meds));

  // Build the new schedule in an empty array. This prevents a deleted
  // Firebase schedule from surviving accidentally in RAM. If the read
  // fails, oldMeds is restored below.
  clearAllSchedules();

  bool networkError = false;
  bool anyValid = false;

  // First try the exact endpoints that match the user's Firebase structure:
  // /schedules/1.json, /schedules/2.json, /schedules/3.json
  for (int i = 1; i <= MAX_MEDS; i++) {
    WiFiClientSecure cli;
    cli.setInsecure();
    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    String url = getFbUrl("schedules/" + String(i) + ".json");

    if (!http.begin(cli, url)) {
      networkError = true;
      Serial.printf("[FB] Could not begin request: %s\n", url.c_str());
      continue;
    }

    http.setTimeout(5000);
    int code = http.GET();

    if (code == 200) {
      String body = http.getString();
      body.trim();

      if (body == "null" || body.length() == 0) {
        Serial.printf("[FB] schedules/%d is empty/null.\n", i);
      } else if (body.startsWith("{")) {
        if (parseOneMed(body, i - 1)) {
          anyValid = true;
        } else {
          // Invalid data is a Firebase-data problem, not a reason to use defaults.
          networkError = true;
        }
      } else {
        Serial.printf("[FB] schedules/%d returned non-object data.\n", i);
        networkError = true;
      }
    } else {
      Serial.printf("[FB] schedules/%d -> HTTP %d (%s)\n", i, code, http.errorToString(code).c_str());
      networkError = true;
    }

    http.end();
  }

  // If direct endpoints were unavailable, try the whole /schedules object.
  // This also handles a Firebase structure where the direct child request fails.
  if (networkError && !anyValid) {
    WiFiClientSecure cli;
    cli.setInsecure();
    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    String url = getFbUrl("schedules.json");

    if (http.begin(cli, url)) {
      http.setTimeout(6000);
      int code = http.GET();

      if (code == 200) {
        String body = http.getString();
        body.trim();

        if (body == "null" || body.length() == 0) {
          Serial.println("[FB] /schedules is empty/null.");
          networkError = false;
        } else if (body.startsWith("{")) {
          networkError = false;
          for (int i = 1; i <= MAX_MEDS; i++) {
            String keyStr = String("\"") + i + "\":";
            int keyPos = body.indexOf(keyStr);
            if (keyPos < 0) continue;

            int bracePos = body.indexOf('{', keyPos);
            if (bracePos < 0) {
              networkError = true;
              continue;
            }

            String obj = extractJsonObject(body, bracePos);
            if (obj.length() > 5) {
              if (parseOneMed(obj, i - 1)) anyValid = true;
              else networkError = true;
            }
          }
        } else {
          networkError = true;
        }
      } else {
        networkError = true;
        Serial.printf("[FB] /schedules -> HTTP %d\n", code);
      }
      http.end();
    } else {
      networkError = true;
    }
  }

  if (networkError) {
    // A failed read must NEVER create an invented schedule.
    // Keep the last successfully received Firebase schedule if one exists.
    memcpy(meds, oldMeds, sizeof(meds));
    firebaseStatus = FB_ERROR;
    firebaseError = "Could not read/validate Firebase schedules";
    Serial.println("[FB] ERROR: keeping last known Firebase schedule; no defaults created.");
    return false;
  }

  // Firebase was successfully read. Missing schedule slots remain cleared.
  // Example: if only schedules/1 exists, schedules/2 and /3 stay inactive.
  for (int i = 0; i < MAX_MEDS; i++) {
    if (meds[i].name[0] != '\0' && meds[i].active) {
      firebaseActiveCount++;
    }
  }

  // Important: an entirely empty Firebase schedule is valid and means NO alarms.
  if (anyValid) {
    firebaseStatus = FB_OK;
    firebaseError = "";
    Serial.println("[FB] Firebase schedules loaded successfully.");
  } else {
    firebaseStatus = FB_EMPTY;
    firebaseError = "No schedules in Firebase";
    Serial.println("[FB] Firebase contains no schedules. All medicine alarms disabled.");
  }

  // Schedule persistence is deliberately NOT used as a source of truth on boot.
  // Firebase remains the authoritative source.
  Serial.println("==================================================");
  for (int i = 0; i < MAX_MEDS; i++) {
    if (meds[i].name[0] != '\0') {
      Serial.printf("  Med[%d] %-20s -> %02d:%02d | %04d-%02d-%02d +%d days | Comp %d | %s | active=%d\n",
                    i, meds[i].name, meds[i].hr, meds[i].mn,
                    meds[i].yr, meds[i].mo, meds[i].dy, meds[i].days,
                    meds[i].comp, meds[i].mealTiming, meds[i].active);
    } else {
      Serial.printf("  Med[%d] -> NOT SET IN FIREBASE\n", i);
    }
  }
  Serial.println("==================================================");
  return true;
}

void syncFB() {
  fetchFirebaseSchedules();
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
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.begin(cli, getFbUrl("LiveState.json"));
  http.addHeader("Content-Type", "application/json");
  http.PUT(buf);
  http.end();
}

// -----------------------------------------------------------------------
// DEFAULT SCHEDULE
// -----------------------------------------------------------------------
// No local/default medicine schedule exists. Firebase is the only source of truth.


// -----------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n=== HydraMedi v4.4 Starting ===");

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

  pinMode(PIN_LED_C1_BEFORE, OUTPUT);
  pinMode(PIN_LED_C1_AFTER,  OUTPUT);
  pinMode(PIN_LED_C2_BEFORE, OUTPUT);
  pinMode(PIN_LED_C2_AFTER,  OUTPUT);
  pinMode(PIN_LED_C3_BEFORE, OUTPUT);
  pinMode(PIN_LED_C3_AFTER,  OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_HX_CLK, LOW);
  
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
    oled.print("HydraMedi v4.4");
    oled.setCursor(20, 36);
    oled.print("Initializing...");
    oled.display();
  } else {
    Serial.println("[WARN] OLED not found!");
  }

  startupAnimation();

  // Servo startup sweep test
  Serial.println("[SERVO] Testing Servos 1, 2, 3...");
  openComp(1); delay(400); closeComp(1); delay(200);
  openComp(2); delay(400); closeComp(2); delay(200);
  openComp(3); delay(400); closeComp(3); delay(200);

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

  // 4. HX711 (Truly non-blocking startup check - prevents scale.begin() hang)
  Serial.println("[INIT] Checking HX711 scale...");
  pinMode(PIN_HX_CLK, OUTPUT);
  digitalWrite(PIN_HX_CLK, LOW);
  pinMode(PIN_HX_DAT, INPUT_PULLUP);

  bool hxHardwareDetected = false;
  unsigned long startCheck = millis();
  while (millis() - startCheck < 120UL) {
    if (digitalRead(PIN_HX_DAT) == LOW) {
      hxHardwareDetected = true;
      break;
    }
    delay(5);
  }

  if (hxHardwareDetected) {
    scale.begin(PIN_HX_DAT, PIN_HX_CLK);
    scale.set_scale(HX711_CALIBRATION);
    resetWeightFilter();
    Serial.println("[OK] HX711 scale hardware detected and initialized.");
  } else {
    Serial.println("[WARN] HX711 scale not connected/ready. Operating without weight sensor dependency.");
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
  // Start with NO schedule. There is deliberately no local/default time.
  clearAllSchedules();
  firebaseStatus = FB_NOT_CHECKED;
  firebaseError = "";

  if (WiFi.isConnected()) {
    syncFB();
  } else {
    firebaseStatus = FB_ERROR;
    firebaseError = "WiFi unavailable";
    Serial.println("[FB] No WiFi. No medicine schedule will be used.");
  }

  Serial.printf("[OK] Setup complete! Local time: %s\n", fmtTime(nowLocal()).c_str());
  for (int i = 0; i < MAX_MEDS; i++) {
    if (meds[i].name[0] != '\0') {
      Serial.printf("  Med[%d] %-20s -> %02d:%02d active=%d %s MEAL\n",
                    i, meds[i].name, meds[i].hr, meds[i].mn,
                    meds[i].active, meds[i].mealTiming);
    } else {
      Serial.printf("  Med[%d] -> NOT SET\n", i);
    }
  }

  timingLEDsOff();
  if (firebaseStatus == FB_ERROR) screenFirebaseError();
  else screenIdle();

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
    for (int i = 0; i < MAX_MEDS; i++) {
      meds[i].takenToday = false;
      lastTriggeredKey[i] = "";
    }
    Serial.println("[RESET] Daily takenToday flags and alarm trigger locks cleared.");
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
        if (firebaseStatus == FB_ERROR) screenFirebaseError();
        else screenIdle();
        Serial.printf("[CLK] %02d:%02d:%02d | Firebase=%d\n",
                      d.hour(), d.minute(), d.second(), (int)firebaseStatus);
      }

      if (ms - tSync >= FIREBASE_SYNC_MS) {
        tSync = ms;
        syncFB();
      }

      // Alarm check: ONLY schedules currently accepted from Firebase can trigger.
      // Date/duration is enforced here as well.
      if (firebaseStatus == FB_OK || firebaseStatus == FB_EMPTY) {
        for (int i = 0; i < MAX_MEDS; i++) {
          Med& m = meds[i];

          String alarmKey = makeAlarmKey(i, m, d);

          if (scheduleIsValidToday(m, d) && !m.takenToday &&
              d.hour() == m.hr && d.minute() == m.mn &&
              lastTriggeredKey[i] != alarmKey) {

          Serial.printf("[ALARM] %s at %02d:%02d | %s MEAL | color=%s\n",
                        m.name, m.hr, m.mn, m.mealTiming, m.color);

          // IMPORTANT: lock this alarm BEFORE changing state.
          // The lock survives Firebase re-syncs during the same minute.
          lastTriggeredKey[i] = alarmKey;
          Serial.printf("[ALARM] Trigger lock set: %s\n", alarmKey.c_str());

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