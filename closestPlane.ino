#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "utilities.h"
#include "TouchDrvGT911.hpp"
#include <Wire.h>
#include <driver/i2s.h>
#include <esp_err.h>
#include <math.h>
#include <vector>
#include <EEPROM.h>
#include <algorithm>

#include "config.h"

#define LILYGO_KB_SLAVE_ADDRESS 0x55

// --- Display & Timing Constants ---
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define REFRESH_INTERVAL_MS 5000
#define WIFI_CONNECT_TIMEOUT_MS 10000

// --- Radar Constants ---
#define EARTH_RADIUS_KM 6371.0
#define BLIP_LIFESPAN_FRAMES 90
#define MAX_BLIPS 20
#define RADAR_CENTER_X 240
#define RADAR_CENTER_Y 120
#define RADAR_RADIUS 100

// --- EEPROM Constants ---
#define EEPROM_SIZE 64
#define EEPROM_ADDR_MAGIC 0
#define EEPROM_ADDR_VOLUME 4
#define EEPROM_ADDR_RANGE_INDEX 8
#define EEPROM_ADDR_SPEED_INDEX 12
#define EEPROM_ADDR_ALERT_DIST 16
#define EEPROM_ADDR_MODE 20
#define EEPROM_ADDR_COMPASS 24
#define EEPROM_MAGIC_NUMBER 0xAD

// --- Display & Input Objects ---
TFT_eSPI display;
TouchDrvGT911 touch;

// --- Volatile variables for ISR-safe encoder reading ---
volatile byte VolumeChange = 0, VolumePush = 0;
volatile byte ChannelChange = 0, ChannelPush = 0;
volatile bool dataConnectionOk = false;

// --- Control State Machine ---
enum ControlMode { VOLUME, SPEED, ALERT, RADAR };
ControlMode currentMode = VOLUME;

// --- Control Variables ---
int beepVolume;

float rangeSteps[] = {5, 10, 25, 50, 100, 150, 200, 300};
const int rangeStepsCount = sizeof(rangeSteps) / sizeof(rangeSteps[0]);
int rangeStepIndex;
float radarRangeKm;

float sweepSpeedSteps[] = {90.0, 180.0, 270.0, 360.0};
const int speedStepsCount = sizeof(sweepSpeedSteps) / sizeof(sweepSpeedSteps[0]);
int sweepSpeedIndex;
float sweepSpeed;

#define INBOUND_ALERT_DISTANCE_KM 5.0
float inboundAlertDistanceKm;

float compassPoints[] = {0.0, 90.0, 180.0, 270.0};
const int compassPointsCount = sizeof(compassPoints) / sizeof(compassPoints[0]);
int compassIndex;
float radarOrientation;

// --- Data Structs ---
struct Aircraft {
  char flight[10];
  double distanceKm;
  double bearing;
  int altitude;
  float groundSpeed;
  float track;
  bool isInbound;
  float minutesToClosest;
  bool isValid;
};

struct RadarBlip {
  int16_t x;
  int16_t y;
  int lifespan;
  bool inbound;
};

// --- Multi-core Shared Data ---
Aircraft lastPingedAircraft; // CHANGED: Stores the last aircraft hit by the sweep
std::vector<Aircraft> trackedAircraft;
std::vector<RadarBlip> activeBlips;
SemaphoreHandle_t dataMutex;
Aircraft closestInboundAircraft;
std::vector<String> alertedFlights;

// --- Animation Variables ---
float sweepAngle = 0.0;
float lastSweepAngle = 0.0;
std::vector<bool> paintedThisTurn;
unsigned long lastFrameTime = 0;

// --- Function Prototypes ---
void saveSettings();
void loadSettings();
void fetchAircraft();
void drawRadarScreen();
void fetchDataTask(void *pvParameters);
void inputTask(void *pvParameters);
void Poweroff(String powermessage);
double haversine(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
void playBeep(int freq, int duration_ms);
void playSiren(int startFreq, int endFreq, int duration_ms);
int getBeepFrequencyForAltitude(int altitude);
double deg2rad(double deg);
void drawDottedCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

// --- BEEP FREQUENCY BANDS ---
#define ALT_LOW_FEET 10000
#define ALT_HIGH_FEET 30000
#define FREQ_LOW 800
#define FREQ_MID 1200
#define FREQ_HIGH 1800

// --- Task for reading keyboard and touchpad input ---
void inputTask(void *pvParameters) {
  int16_t lastX = -1, lastY = -1;
  for (;;) {
    // Keyboard handling
    Wire.requestFrom(LILYGO_KB_SLAVE_ADDRESS, 1);
    while (Wire.available()) {
      char c = Wire.read();
      if (c == 'w') VolumeChange = 1;
      else if (c == 's') VolumeChange = 2;
      else if (c == 'd') ChannelChange = 1;
      else if (c == 'a') ChannelChange = 2;
      else if (c == 'm' || c == 'M') ChannelPush = 1;
      else if (c == 'p' || c == 'P') VolumePush = 2;
    }

    // Touchpad handling
    if (touch.isPressed()) {
      int16_t x[1], y[1];
      if (touch.getPoint(x, y, 1)) {
        if (lastX >= 0) {
          int dx = x[0] - lastX;
          int dy = y[0] - lastY;
          if (dx > 5) ChannelChange = 1;
          else if (dx < -5) ChannelChange = 2;
          if (dy > 5) VolumeChange = 2;
          else if (dy < -5) VolumeChange = 1;
        }
        lastX = x[0];
        lastY = y[0];
      }
    } else {
      lastX = lastY = -1;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Determines beep pitch based on altitude
int getBeepFrequencyForAltitude(int altitude) {
  if (altitude < 0) { // Default for unknown altitude
    return FREQ_MID;
  }
  if (altitude < ALT_LOW_FEET) {
    return FREQ_LOW;
  } else if (altitude < ALT_HIGH_FEET) {
    return FREQ_MID;
  } else {
    return FREQ_HIGH;
  }
}

// --- TASK FOR CORE 0: DATA FETCHING ---
void fetchDataTask(void *pvParameters) {
  for (;;) {
    fetchAircraft();
    vTaskDelay(REFRESH_INTERVAL_MS / portTICK_PERIOD_MS);
  }
}

void setup() {
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);
  Serial.begin(115200);
  Serial.println("Booting Multi-Target Radar (Enhanced)");

  pinMode(BOARD_POWERON, OUTPUT);
  digitalWrite(BOARD_POWERON, HIGH);

  loadSettings();
  dataMutex = xSemaphoreCreateMutex();
  lastPingedAircraft.isValid = false; // CHANGED: Initialize new display variable
  closestInboundAircraft.isInbound = false;
  closestInboundAircraft.isValid = false;

  display.begin();
  display.setRotation(1);
  display.fillScreen(TFT_BLACK);
  display.setTextSize(1);
  display.setTextColor(TFT_WHITE);
  display.setCursor(0, 0);
  display.println("Multi-Target Radar");
  display.setCursor(0, 16);
  display.println("Connecting WiFi...");

  pinMode(BOARD_BL_PIN, OUTPUT);
  digitalWrite(BOARD_BL_PIN, HIGH);

  Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);
  touch.setPins(-1, BOARD_TOUCH_INT);
  touch.begin(Wire, GT911_SLAVE_ADDRESS_L);
  touch.setMaxCoordinates(SCREEN_WIDTH, SCREEN_HEIGHT);
  touch.setSwapXY(true);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print('.');
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected.");
    WiFi.setSleep(false);
  } else {
    Serial.println("\nWiFi connection failed.");
    display.fillRect(0, 16, SCREEN_WIDTH, 16, TFT_BLACK);
    display.setCursor(0, 16);
    display.println("WiFi Failed");
  }

  i2s_config_t i2s_config = {.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = 44100, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, .communication_format = I2S_COMM_FORMAT_STAND_I2S, .intr_alloc_flags = 0, .dma_buf_count = 8, .dma_buf_len = 64, .use_apll = false, .tx_desc_auto_clear = true, .fixed_mclk = 0};
  i2s_pin_config_t pin_config = {.bck_io_num = I2S_BCLK_PIN, .ws_io_num = I2S_LRCLK_PIN, .data_out_num = I2S_DOUT_PIN, .data_in_num = I2S_PIN_NO_CHANGE};
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  xTaskCreatePinnedToCore(fetchDataTask, "FetchData", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(inputTask, "Input", 4096, NULL, 3, NULL, 1);
}

// --- MAIN LOOP ON CORE 1 (Graphics and Logic) ---
void loop() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastFrameTime;

  byte localVolumeChange = VolumeChange;
  byte localVolumePush = VolumePush;
  byte localChannelChange = ChannelChange;
  byte localChannelPush = ChannelPush;
  if (localVolumeChange != 0) VolumeChange = 0;
  if (localVolumePush != 0) VolumePush = 0;
  if (localChannelChange != 0) ChannelChange = 0;
  if (localChannelPush != 0) ChannelPush = 0;

  if (localChannelPush == 1) {
    if (currentMode == VOLUME) currentMode = SPEED;
    else if (currentMode == SPEED) currentMode = ALERT;
    else if (currentMode == ALERT) currentMode = RADAR;
    else currentMode = VOLUME;
  }

  if (localVolumeChange != 0) {
    if (currentMode == VOLUME) {
      beepVolume += (localVolumeChange == 1) ? 1 : -1;
      beepVolume = constrain(beepVolume, 0, 20);
    } else if (currentMode == SPEED) {
      sweepSpeedIndex += (localVolumeChange == 1) ? 1 : -1;
      sweepSpeedIndex = constrain(sweepSpeedIndex, 0, speedStepsCount - 1);
      sweepSpeed = sweepSpeedSteps[sweepSpeedIndex];
    } else if (currentMode == ALERT) {
      inboundAlertDistanceKm += (localVolumeChange == 1) ? 1 : -1;
      inboundAlertDistanceKm = constrain(inboundAlertDistanceKm, 1.0, 50.0);
    } else if (currentMode == RADAR) {
      compassIndex += (localVolumeChange == 1) ? 1 : -1;
      if (compassIndex < 0) compassIndex = compassPointsCount - 1;
      if (compassIndex >= compassPointsCount) compassIndex = 0;
      radarOrientation = compassPoints[compassIndex];
      activeBlips.clear();
      paintedThisTurn.assign(trackedAircraft.size(), false);
      lastSweepAngle = sweepAngle;
    }
  }
  
  if (localVolumePush == 2) { Poweroff("  Goodbye"); }
  
  if (localChannelChange != 0) {
    rangeStepIndex += (localChannelChange == 1) ? 1 : -1;
    rangeStepIndex = constrain(rangeStepIndex, 0, rangeStepsCount - 1);
    radarRangeKm = rangeSteps[rangeStepIndex];
    
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    activeBlips.clear(); 
    std::vector<Aircraft> filteredAircraft;
    // REMOVED: Logic for finding closest aircraft, not needed here anymore
    for (const auto& ac : trackedAircraft) {
      if (ac.distanceKm < radarRangeKm) {
        filteredAircraft.push_back(ac);
      }
    }
    trackedAircraft = filteredAircraft;
    if (trackedAircraft.empty()) { // If no planes left, invalidate display
        lastPingedAircraft.isValid = false;
    }
    paintedThisTurn.assign(trackedAircraft.size(), false);
    xSemaphoreGive(dataMutex);
  }
  


  if (deltaTime > 0) {
    lastSweepAngle = sweepAngle;
    sweepAngle += (sweepSpeed / 1000.0) * deltaTime;
    if (sweepAngle >= 360.0) {
      sweepAngle = fmod(sweepAngle, 360.0);
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      paintedThisTurn.assign(trackedAircraft.size(), false);
      xSemaphoreGive(dataMutex);
    }
  }

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  for (int i = 0; i < trackedAircraft.size(); i++) {
    if (i < paintedThisTurn.size() && !paintedThisTurn[i]) {
      double targetBearing = trackedAircraft[i].bearing - radarOrientation;
      if (targetBearing < 0) targetBearing += 360.0;
      bool bearingCrossed = (lastSweepAngle < targetBearing && sweepAngle >= targetBearing);
      if (lastSweepAngle > sweepAngle && (targetBearing > lastSweepAngle || targetBearing <= sweepAngle)) {
        bearingCrossed = true;
      }
      if (bearingCrossed) {
        double angleRad = targetBearing * PI / 180.0;
        double realDistance = trackedAircraft[i].distanceKm;
        float screenRadius = map(realDistance, 0, radarRangeKm, 0, RADAR_RADIUS);
        int16_t newBlipX = RADAR_CENTER_X + screenRadius * sin(angleRad);
        int16_t newBlipY = RADAR_CENTER_Y - screenRadius * cos(angleRad);
        activeBlips.push_back({newBlipX, newBlipY, BLIP_LIFESPAN_FRAMES, trackedAircraft[i].isInbound});
        if (activeBlips.size() > MAX_BLIPS) {
          activeBlips.erase(activeBlips.begin());
        }
        
        lastPingedAircraft = trackedAircraft[i]; // NEW: Update display data with current aircraft
        
        paintedThisTurn[i] = true;
        int freq = getBeepFrequencyForAltitude(trackedAircraft[i].altitude);
        playBeep(freq, 20);
      }
    }
  }
  
  for (auto it = activeBlips.begin(); it != activeBlips.end(); ) {
    it->lifespan--;
    if (it->lifespan <= 0) { it = activeBlips.erase(it); } 
    else { ++it; }
  }
  xSemaphoreGive(dataMutex);

  drawRadarScreen();
  lastFrameTime = currentTime;
}

void Poweroff(String powermessage) {
  Serial.println("Powering off. Saving settings...");
  saveSettings();
  i2s_driver_uninstall(I2S_NUM_0);
  display.fillScreen(TFT_BLACK);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(powermessage);
  delay(1000);
  digitalWrite(19, LOW);
}

void saveSettings() {
  EEPROM.put(EEPROM_ADDR_VOLUME, beepVolume);
  EEPROM.put(EEPROM_ADDR_RANGE_INDEX, rangeStepIndex);
  EEPROM.put(EEPROM_ADDR_SPEED_INDEX, sweepSpeedIndex);
  EEPROM.put(EEPROM_ADDR_ALERT_DIST, inboundAlertDistanceKm);
  EEPROM.put(EEPROM_ADDR_MODE, currentMode);
  EEPROM.put(EEPROM_ADDR_COMPASS, compassIndex);
  EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_NUMBER);
  EEPROM.commit();
  Serial.println("Settings saved to EEPROM.");
}

void loadSettings() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(EEPROM_ADDR_MAGIC) == EEPROM_MAGIC_NUMBER) {
    Serial.println("Loading settings from EEPROM...");
    EEPROM.get(EEPROM_ADDR_VOLUME, beepVolume);
    EEPROM.get(EEPROM_ADDR_RANGE_INDEX, rangeStepIndex);
    EEPROM.get(EEPROM_ADDR_SPEED_INDEX, sweepSpeedIndex);
    EEPROM.get(EEPROM_ADDR_ALERT_DIST, inboundAlertDistanceKm);
    EEPROM.get(EEPROM_ADDR_MODE, currentMode);
    EEPROM.get(EEPROM_ADDR_COMPASS, compassIndex);
    beepVolume = constrain(beepVolume, 0, 20);
    rangeStepIndex = constrain(rangeStepIndex, 0, rangeStepsCount - 1);
    sweepSpeedIndex = constrain(sweepSpeedIndex, 0, speedStepsCount - 1);
    inboundAlertDistanceKm = constrain(inboundAlertDistanceKm, 1.0f, 50.0f);
    currentMode = (ControlMode)constrain((int)currentMode, 0, RADAR);
    compassIndex = constrain(compassIndex, 0, compassPointsCount - 1);
  } else {
    Serial.println("First run or invalid EEPROM data. Setting defaults.");
    beepVolume = 10;
    rangeStepIndex = 3;
    sweepSpeedIndex = 1;
    inboundAlertDistanceKm = INBOUND_ALERT_DISTANCE_KM;
    currentMode = VOLUME;
    compassIndex = 0;
    saveSettings();
  }
  radarRangeKm = rangeSteps[rangeStepIndex];
  sweepSpeed = sweepSpeedSteps[sweepSpeedIndex];
  radarOrientation = compassPoints[compassIndex];
  Serial.printf("Loaded Volume: %d\n", beepVolume);
  Serial.printf("Loaded Range: %.0f km\n", radarRangeKm);
  Serial.printf("Loaded Speed: %.1f deg/s\n", sweepSpeed);
  Serial.printf("Loaded Alert Dist: %.1f km\n", inboundAlertDistanceKm);
}

void drawRadarScreen() {
  display.fillScreen(TFT_BLACK);
  display.setTextSize(1);
  display.setTextColor(TFT_WHITE);

  // Settings summary row
  display.setCursor(0, 0);
  if (currentMode == VOLUME) display.setTextColor(TFT_BLACK, TFT_WHITE);
  display.print("V");
  display.setTextColor(TFT_WHITE);
  display.print(":");
  display.print(beepVolume);
  display.print(" ");
  if (currentMode == SPEED) display.setTextColor(TFT_BLACK, TFT_WHITE);
  display.print("S");
  display.setTextColor(TFT_WHITE);
  display.print(":");
  display.print(sweepSpeed, 0);
  display.print(" ");
  if (currentMode == ALERT) display.setTextColor(TFT_BLACK, TFT_WHITE);
  display.print("P");
  display.setTextColor(TFT_WHITE);
  display.print(":");
  display.print(inboundAlertDistanceKm, 0);
  display.print("k");

  Aircraft currentAircraftToDisplay; // CHANGED
  Aircraft currentClosestInbound;
  std::vector<RadarBlip> currentBlips;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  currentAircraftToDisplay = lastPingedAircraft; // CHANGED: Get last pinged aircraft
  currentBlips = activeBlips;
  currentClosestInbound = closestInboundAircraft;
  xSemaphoreGive(dataMutex);

  // CHANGED: All display logic now uses currentAircraftToDisplay
  if (currentAircraftToDisplay.isValid) {
    display.setCursor(0, 8);
    display.print("Flt: ");
    display.println(strlen(currentAircraftToDisplay.flight) > 0 ? currentAircraftToDisplay.flight : "------");
    display.print("Dst: ");
    display.print(currentAircraftToDisplay.distanceKm, 1);
    display.println("km");
    display.print("Alt: ");
    if (currentAircraftToDisplay.altitude >= 0) {
      display.print(currentAircraftToDisplay.altitude);
      display.println("ft");
    } else {
      display.println("-----");
    }
    display.print("Spd: ");
    if (currentAircraftToDisplay.groundSpeed >= 0) {
        display.print(currentAircraftToDisplay.groundSpeed, 0);
        display.println("kt");
    } else {
        display.println("---");
    }
    display.print("Rng: ");
    display.print(radarRangeKm, 0);
    display.println("km");
  } else {
    display.setCursor(0, 8);
    display.println("Scanning...");
    display.setCursor(0, 24);
    display.print("Range:");
    display.print(radarRangeKm, 0);
    display.println("km");
  }

  if (currentClosestInbound.isInbound && currentClosestInbound.minutesToClosest >= 0) {
    display.setCursor(0, 56);
    display.print("ETA: ");
    display.print(currentClosestInbound.minutesToClosest, 1);
    display.print("m");
  }

  // Draw WiFi signal strength meter
  int16_t wifiX = SCREEN_WIDTH - 14;
  int16_t wifiY = 8;
  int bars = 0;
  if (WiFi.status() == WL_CONNECTED) {
    long rssi = WiFi.RSSI();
    if (rssi > -55)      bars = 4;
    else if (rssi > -65) bars = 3;
    else if (rssi > -75) bars = 2;
    else if (rssi > -85) bars = 1;
  }
  for (int i = 0; i < 4; i++) {
    int barHeight = (i + 1) * 2;
    int barX = wifiX + i * 3;
    int barY = wifiY - barHeight;
    if (i < bars) {
      display.fillRect(barX, barY, 2, barHeight, TFT_WHITE);
    } else {
      display.drawRect(barX, barY, 2, barHeight, TFT_WHITE);
    }
  }

  // Dump1090 connection status indicator
  int16_t dumpX = wifiX - 6;
  int16_t dumpY = wifiY - 2;
  if (dataConnectionOk) {
    display.fillCircle(dumpX, dumpY, 2, TFT_WHITE);
  } else {
    display.drawCircle(dumpX, dumpY, 2, TFT_WHITE);
    display.drawLine(dumpX - 2, dumpY - 2, dumpX + 2, dumpY + 2, TFT_WHITE);
    display.drawLine(dumpX - 2, dumpY + 2, dumpX + 2, dumpY - 2, TFT_WHITE);
  }

  display.drawCircle(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_RADIUS, TFT_WHITE);
  display.setCursor(RADAR_CENTER_X - 3, RADAR_CENTER_Y - RADAR_RADIUS - 9);
  if (currentMode == RADAR) display.setTextColor(TFT_BLACK, TFT_WHITE);
  const char compassLetters[] = {'N','E','S','W'};
  display.print(compassLetters[compassIndex]);
  if (currentMode == RADAR) display.setTextColor(TFT_WHITE);

  drawDottedCircle(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_RADIUS * 2 / 3, TFT_WHITE);
  drawDottedCircle(RADAR_CENTER_X, RADAR_CENTER_Y, RADAR_RADIUS * 1 / 3, TFT_WHITE);

  unsigned long flashPhase = millis() / 300;
  for (const auto& blip : currentBlips) {
    if (blip.inbound && (flashPhase % 2)) continue;
    if (blip.lifespan > (BLIP_LIFESPAN_FRAMES * 2 / 3)) {
      display.fillCircle(blip.x, blip.y, 3, TFT_WHITE);
    } else if (blip.lifespan > (BLIP_LIFESPAN_FRAMES * 1 / 3)) {
      display.fillCircle(blip.x, blip.y, 2, TFT_WHITE);
    } else {
      display.fillCircle(blip.x, blip.y, 1, TFT_WHITE);
    }
  }

  double sweepRad = sweepAngle * PI / 180.0;
  int16_t sweepX = RADAR_CENTER_X + (RADAR_RADIUS - 1) * sin(sweepRad);
  int16_t sweepY = RADAR_CENTER_Y - (RADAR_RADIUS - 1) * cos(sweepRad);
  display.drawLine(RADAR_CENTER_X, RADAR_CENTER_Y, sweepX, sweepY, TFT_WHITE);
}

void fetchAircraft() {
  HTTPClient http;
  char url[160];
  snprintf(url, sizeof(url), "http://%s:%d/dump1090-fa/data/aircraft.json", DUMP1090_SERVER, DUMP1090_PORT);

  if (!http.begin(url)) {
    dataConnectionOk = false;
    return;
  }

  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    DynamicJsonDocument filter(512);
    filter["aircraft"][0]["lat"] = true;
    filter["aircraft"][0]["lon"] = true;
    filter["aircraft"][0]["flight"] = true;
    filter["aircraft"][0]["alt_baro"] = true;
    filter["aircraft"][0]["gs"] = true;
    filter["aircraft"][0]["track"] = true;

    DynamicJsonDocument doc(8192); 
    if (deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter)) == DeserializationError::Ok) {
      dataConnectionOk = true;
      JsonArray arr = doc["aircraft"].as<JsonArray>();
      
      std::vector<Aircraft> planesInRange;
      Aircraft bestInbound;
      bestInbound.isInbound = false;
      bestInbound.isValid = false;

      for (JsonObject plane : arr) {
        if (plane.containsKey("lat") && plane.containsKey("lon")) {
          double dist = haversine(USER_LAT, USER_LON, plane["lat"], plane["lon"]);

          if (isnan(dist) || isinf(dist)) {
            continue;
          }
          
          if (dist < radarRangeKm) {
            Aircraft ac;
            const char* flightStr = plane["flight"].as<const char*>();
            if (flightStr) {
                strncpy(ac.flight, flightStr, sizeof(ac.flight) - 1);
                ac.flight[sizeof(ac.flight) - 1] = '\0';
            } else {
                ac.flight[0] = '\0';
            }

            ac.distanceKm = dist;
            ac.bearing = calculateBearing(USER_LAT, USER_LON, plane["lat"], plane["lon"]);
            
            if (plane.containsKey("alt_baro")) {
              JsonVariant alt = plane["alt_baro"];
              if (alt.is<int>()) {
                ac.altitude = alt.as<int>();
              } else if (alt.is<const char*>() && strcmp(alt.as<const char*>(), "ground") == 0) {
                ac.altitude = 0;
              } else {
                ac.altitude = -1;
              }
            } else {
                ac.altitude = -1;
            }

            if (plane.containsKey("gs")) {
                ac.groundSpeed = plane["gs"].as<float>();
            } else {
                ac.groundSpeed = -1.0;
            }
            if (plane.containsKey("track")) {
                ac.track = plane["track"].as<float>();
            } else {
                ac.track = -1.0;
            }

            if (isnan(ac.bearing) || isinf(ac.bearing)) {
                continue;
            }

            // Inbound prediction
            ac.isInbound = false;
            ac.minutesToClosest = -1.0;
            if (ac.track >= 0 && ac.groundSpeed > 0) {
              double toBase = fmod(ac.bearing + 180.0, 360.0);
              double angleDiff = fabs(ac.track - toBase);
              if (angleDiff > 180.0) angleDiff = 360.0 - angleDiff;
              double crossTrack = ac.distanceKm * sin(deg2rad(angleDiff));
              double alongTrack = ac.distanceKm * cos(deg2rad(angleDiff));
              if (angleDiff < 90.0 && fabs(crossTrack) < inboundAlertDistanceKm) {
                ac.isInbound = true;
                double speedKmMin = ac.groundSpeed * 1.852 / 60.0;
                if (speedKmMin > 0) {
                  ac.minutesToClosest = alongTrack / speedKmMin;
                }
                bool already = std::find(alertedFlights.begin(), alertedFlights.end(), String(ac.flight)) != alertedFlights.end();
                if (!already) {
                  alertedFlights.push_back(String(ac.flight));
                  playSiren(800, 1800, 500);
                }
              } else {
                auto it = std::find(alertedFlights.begin(), alertedFlights.end(), String(ac.flight));
                if (it != alertedFlights.end()) alertedFlights.erase(it);
              }
            }

            ac.isValid = true;
            planesInRange.push_back(ac);
            if (ac.isInbound && ac.minutesToClosest >= 0) {
              if (!bestInbound.isInbound || ac.minutesToClosest < bestInbound.minutesToClosest) {
                bestInbound = ac;
              }
            }
          }
        }
      }

      xSemaphoreTake(dataMutex, portMAX_DELAY);
      trackedAircraft = planesInRange;
      if (bestInbound.isInbound) {
        closestInboundAircraft = bestInbound;
      } else {
        closestInboundAircraft.isInbound = false;
        closestInboundAircraft.isValid = false;
      }
      xSemaphoreGive(dataMutex);
    } else {
      dataConnectionOk = false;
    }
  } else {
    dataConnectionOk = false;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    trackedAircraft.clear();
    lastPingedAircraft.isValid = false; // Invalidate display on connection loss
    xSemaphoreGive(dataMutex);
  }
  http.end();
}

void playBeep(int freq, int duration_ms) {
  long amplitude = map(beepVolume, 0, 20, 0, 25000);
  const int sampleRate = 44100; int samples = sampleRate * duration_ms / 1000; size_t bytes_written;
  for (int i = 0; i < samples; i++) {
    int16_t sample = (int16_t)(sin(2 * PI * freq * i / sampleRate) * amplitude);
    i2s_write(I2S_NUM_0, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);
  }
}

void playSiren(int startFreq, int endFreq, int duration_ms) {
  long amplitude = map(beepVolume, 0, 20, 0, 25000);
  const int sampleRate = 44100;
  int samples = sampleRate * duration_ms / 1000;
  size_t bytes_written;
  for (int i = 0; i < samples; i++) {
    float progress = (float)i / samples;
    float freq = startFreq + (endFreq - startFreq) * progress;
    int16_t sample = (int16_t)(sin(2 * PI * freq * i / sampleRate) * amplitude);
    i2s_write(I2S_NUM_0, &sample, sizeof(sample), &bytes_written, portMAX_DELAY);
  }
}

double deg2rad(double deg) { return deg * PI / 180.0; }

double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dLat = deg2rad(lat2 - lat1); double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS_KM * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
  double lonDiff = deg2rad(lon2 - lon1); lat1 = deg2rad(lat1); lat2 = deg2rad(lat2);
  double y = sin(lonDiff) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lonDiff);
  double bearing = atan2(y, x);
  bearing = fmod((bearing * 180.0 / PI + 360.0), 360.0);
  return bearing;
}

// Custom function to draw a dotted circle
void drawDottedCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  for (int i = 0; i < 360; i += 30) {
    double angleRad = i * PI / 180.0;
    int16_t x = x0 + r * sin(angleRad);
    int16_t y = y0 - r * cos(angleRad);
    display.drawPixel(x, y, color);
  }
}

