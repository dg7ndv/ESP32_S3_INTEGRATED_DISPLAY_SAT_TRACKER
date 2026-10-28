#include <Arduino.h>
#include <LittleFS.h>
#include <lvgl.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "HB9IIUdisplayInit.h"
#include "lv_driver.h"
#include "boot_manager.h"
#include "sat_tracker.h"
#include "screen_manager.h"
#include "myconfig.h"
#include "LvglScreenshot.h"
#include "orientation_check.h"


LGFX tft;
static LvglScreenshot screenshot;
static WebServer httpServer(8080);

static void serveTracking() {
    const auto& state = SatTracker::getState();
    httpServer.sendHeader("Cache-Control", "no-store");

    if (!state.tleLoaded || state.name[0] == '\0' ||
        !isfinite(state.azimuth) || !isfinite(state.elevation)) {
        httpServer.send(503, "application/json", "{\"error\":\"Tracking data unavailable\"}");
        return;
    }

    JsonDocument response;
    response["satellite"] = state.name;
    response["azimuth"] = state.azimuth;
    response["elevation"] = state.elevation;

    String body;
    serializeJson(response, body);
    httpServer.send(200, "application/json", body);
}

// Set active satellite and/or observer location, e.g.
//   /api/config?sat=25544   /api/config?lat=47.4&lon=8.5   (GET or POST)
static void serveConfig() {
    httpServer.sendHeader("Cache-Control", "no-store");
    bool hasSat = httpServer.hasArg("sat");
    bool hasLat = httpServer.hasArg("lat"), hasLon = httpServer.hasArg("lon");

    if (!hasSat && !hasLat && !hasLon) {
        httpServer.send(400, "application/json", "{\"error\":\"Use sat=<NORAD> and/or lat=&lon=\"}");
        return;
    }
    if (hasLat != hasLon) {
        httpServer.send(400, "application/json", "{\"error\":\"lat and lon must be given together\"}");
        return;
    }

    uint32_t satId = hasSat ? (uint32_t)httpServer.arg("sat").toInt() : SatTracker::getState().noradId;
    if (hasSat && satId == 0) {
        httpServer.send(400, "application/json", "{\"error\":\"Invalid NORAD id\"}");
        return;
    }

    if (hasLat) {
        float lat = httpServer.arg("lat").toFloat(), lon = httpServer.arg("lon").toFloat();
        if (lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f) {
            httpServer.send(400, "application/json", "{\"error\":\"lat +-90, lon +-180\"}");
            return;
        }
        NVSConfig::LocationData cur = NVSConfig::loadLocation();
        NVSConfig::saveLocation(lat, lon, cur.valid ? cur.timezone : "UTC");
    }

    uint32_t prevSat = SatTracker::getState().noradId;
    bool ok = satId > 0 ? SatTracker::begin(satId) : true;   // reload tracker (also applies new site)
    if (!ok) {
        if (prevSat > 0 && SatTracker::begin(prevSat)) SatTracker::update();   // keep the previous satellite active
        httpServer.send(404, "application/json", "{\"error\":\"No TLE cached for this NORAD id\"}");
        return;
    }
    if (hasSat) NVSConfig::saveSelectedSat(satId);
    SatTracker::update();   // fill az/el right away instead of reporting zeros
    httpServer.send(200, "application/json", "{\"ok\":true}");
}

static void show_splash() {
    File root = LittleFS.open("/");
    File f = root.openNextFile();
    Serial.println("[splash] LittleFS root:");
    while (f) {
        Serial.printf("  %s  (%u bytes)\n", f.name(), f.size());
        f = root.openNextFile();
    }
    if (!LittleFS.exists("/splash.jpg")) {
        Serial.println("[splash] /splash.jpg not found.");
        return;
    }
    bool ok = tft.drawJpgFile(LittleFS, "/splash.jpg", 0, 0, 800, 480);
    Serial.printf("[splash] drawJpgFile: %s\n", ok ? "OK" : "FAILED");
    if (ok) {
        tft.setTextDatum(lgfx::datum_t::bottom_right);
        tft.setFont(&fonts::DejaVu18);
        tft.setTextColor(0xCCCCCC);
        tft.drawString("v" FIRMWARE_VERSION, 790, 472);
    }
}


void setup() {
    uint32_t t_setup0 = millis();
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
    delay(500);
    Serial.println("\n=== ESP32-S3 Satellite Tracker ===");

    initTFT();
    runOrientationCheck();

    if (!LittleFS.begin(false, "/littlefs", 10, "littlefs")) {
        Serial.println("LittleFS mount failed.");
    } else {
        Serial.println("LittleFS mounted.");
        show_splash();
    }

    uint32_t t_boot0 = millis();
    BootManager::run();
    Serial.printf("[perf] BootManager::run()            %lu ms\n", millis() - t_boot0);

    // ArduinoOTA — allows "Upload" from PlatformIO env:DISPLAY_OTA over WiFi
    ArduinoOTA.setHostname(WIFI_HOSTNAME);  // must match boot_manager MDNS.begin() hostname
    // ArduinoOTA.setPassword("secret");  // uncomment to require a password
    ArduinoOTA.begin();
    Serial.printf("[ota] ArduinoOTA ready  (%s.local:3232)\n", WIFI_HOSTNAME);

    uint32_t t_lv0 = millis();
    lv_init();
    lvgl_setup();
    Serial.println("LVGL ready.");
    Serial.printf("[perf] LVGL init + driver setup       %lu ms\n", millis() - t_lv0);

    if (screenshot.begin(800, 480, &httpServer)) {
        screenshot.setPreServeFn([](uint16_t* buf, uint16_t w, uint16_t h) {
            tft.readRect(0, 0, w, h, buf);
        });
    }
    httpServer.on("/api/tracking", HTTP_GET, serveTracking);
    httpServer.on("/api/config", serveConfig);
    httpServer.begin();
    Serial.printf("[http] Tracking API ready at http://%s.local:8080/api/tracking\n", WIFI_HOSTNAME);

    uint32_t satId = NVSConfig::loadSelectedSat(DEFAULT_SAT_ID);
    BootManager::drawStatus("Computing orbits...   Please wait...", 0x4A9ECC);
    uint32_t t_sat0 = millis();
    SatTracker::begin(satId);
    Serial.printf("[perf] SatTracker::begin()            %lu ms\n", millis() - t_sat0);
    SatTracker::startSkyTask();
    BootManager::drawStatus("Building interface...   Please wait...", 0x4A9ECC);

    uint32_t t_ui0 = millis();
    ScreenManager::build(lv_scr_act());
    Serial.printf("[perf] ScreenManager::build()         %lu ms\n", millis() - t_ui0);

    Serial.printf("[perf] setup() total                  %lu ms\n", millis() - t_setup0);
    Serial.println("UI ready. Entering loop.");
}

void loop() {
    ArduinoOTA.handle();
    lv_timer_handler();
    SatTracker::runPassCompute();      // next-pass search for active sat
    httpServer.handleClient();

    delay(5);
}
