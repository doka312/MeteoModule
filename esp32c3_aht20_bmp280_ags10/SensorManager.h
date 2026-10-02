#pragma once

#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

// Defaults in case the sketch does not define them
#ifndef ENABLE_AGS10
#define ENABLE_AGS10    false
#endif
#ifndef AGS10_PIN_SDA
#define AGS10_PIN_SDA   5
#endif
#ifndef AGS10_PIN_SCL
#define AGS10_PIN_SCL   6
#endif
#ifndef AGS10_I2C_FREQ
#define AGS10_I2C_FREQ  10000
#endif

#if ENABLE_AGS10
#include "AGS10.h"
#endif

class SensorManager {
public:
    float    temperature = 0.0f;
    float    humidity    = 0.0f;
    float    pressure    = 0.0f;   // hPa
    int32_t  tvoc        = -1;     // ppb (AGS10); -1 = no data (absent / warming up / error)
    bool     ahtOk       = false;
    bool     bmpOk       = false;
    bool     agsOk       = false;
    bool     agsWarming  = false;

    bool begin() {
        bool ok = true;

        // AHT20
        if (!aht.begin()) {
            Serial.println("[SENSOR] AHT20 not found!");
            ok = false;
        } else {
            ahtOk = true;
            Serial.println("[AHT20]  OK");
        }

        // BMP280 — try both addresses
        if (!bmp.begin(0x76)) {
            if (!bmp.begin(0x77)) {
                Serial.println("[SENSOR] BMP280 not found!");
                ok = false;
            } else { bmpOk = true; }
        } else { bmpOk = true; }

        if (bmpOk) {
            bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                            Adafruit_BMP280::SAMPLING_X2,
                            Adafruit_BMP280::SAMPLING_X16,
                            Adafruit_BMP280::FILTER_X16,
                            Adafruit_BMP280::STANDBY_MS_500);
            Serial.println("[BMP280] OK");
        }

        // AGS10 (optional, own software I2C bus)
        #if ENABLE_AGS10
        probeAgs10();
        #endif

        return ok;
    }

    void update() {
        // AHT20
        if (ahtOk) {
            sensors_event_t humEvent, tempEvent;
            if (aht.getEvent(&humEvent, &tempEvent)) {
                temperature = tempEvent.temperature;
                humidity    = humEvent.relative_humidity;
            }
        }

        // BMP280
        if (bmpOk) {
            pressure = bmp.readPressure() / 100.0f;
        }

        // AGS10
        #if ENABLE_AGS10
        updateAgs10();
        #endif
    }

private:
    Adafruit_AHTX0   aht;
    Adafruit_BMP280  bmp;

    #if ENABLE_AGS10
    AGS10         ags{AGS10_PIN_SDA, AGS10_PIN_SCL, AGS10_I2C_FREQ};
    unsigned long agsLastProbe = 0;
    uint8_t       agsErrors    = 0;

    static constexpr unsigned long AGS_REPROBE_MS  = 30000; // retry if not found
    static constexpr uint8_t       AGS_MAX_ERRORS  = 5;     // consecutive failures → "lost"

    void probeAgs10() {
        agsLastProbe = millis();
        agsOk = ags.begin();
        agsErrors = 0;
        if (agsOk) Serial.printf("[AGS10]  OK (fw 0x%02X, SDA=%d SCL=%d)\n",
                                 ags.version, AGS10_PIN_SDA, AGS10_PIN_SCL);
        else       Serial.printf("[AGS10]  not found (SDA=%d SCL=%d)\n",
                                 AGS10_PIN_SDA, AGS10_PIN_SCL);
    }

    void updateAgs10() {
        if (!agsOk) {
            tvoc = -1;
            agsWarming = false;
            if (millis() - agsLastProbe >= AGS_REPROBE_MS) probeAgs10();
            return;
        }

        uint32_t ppb;
        switch (ags.readTVOC(ppb)) {
            case AGS10::VALUE:
                if (agsWarming) Serial.println("[AGS10]  warm-up finished");
                tvoc = (int32_t)ppb;
                agsWarming = false;
                agsErrors = 0;
                break;

            case AGS10::WARMING:
                tvoc = -1;
                agsWarming = true;
                agsErrors = 0;
                break;

            case AGS10::FAILED:
                // Keep the last good value on a single glitch
                if (++agsErrors >= AGS_MAX_ERRORS) {
                    Serial.println("[AGS10]  no response, will retry");
                    agsOk = false;
                    tvoc = -1;
                    agsLastProbe = millis();
                }
                break;

            case AGS10::TOO_SOON:
                break;
        }
    }
    #endif
};
