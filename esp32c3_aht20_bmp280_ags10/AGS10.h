#pragma once

#include <Arduino.h>
#include "SoftI2C.h"

// ══════════════════════════════════════════════════════════
//  AGS10 TVOC sensor driver (per Aosong datasheet, 2022)
//  ────────────────────────────────────────────────────────
//  - I2C address 0x1A, bus clock <= 15 kHz (own soft bus)
//  - >= 30 ms between any two commands
//  - data acquisition (reg 0x00) not more often than 1.5 s
//  - every reply is 5 bytes: Data1..Data4 + CRC8
//    (poly 0x31, init 0xFF) over Data1..Data4
//  - reg 0x00: [status, tvoc_hi, tvoc_mid, tvoc_lo, crc]
//      status bit0 (RDY): 1 = warming up / no data yet
//  - reg 0x11: firmware version in Data4
//
//  This driver only READS registers 0x00 and 0x11. It never
//  sends the zero-point calibration (0x01) or the address
//  change (0x21) commands.
// ══════════════════════════════════════════════════════════

class AGS10 {
public:
    static constexpr uint8_t  ADDR          = 0x1A;
    static constexpr uint8_t  REG_TVOC      = 0x00;
    static constexpr uint8_t  REG_VERSION   = 0x11;
    static constexpr uint32_t CMD_GAP_MS    = 30;
    static constexpr uint32_t MIN_ACQ_MS    = 1500;

    enum Result {
        VALUE,     // ppb holds a fresh value
        WARMING,   // sensor is pre-heating, no valid value yet
        TOO_SOON,  // called earlier than 1.5 s after the last read — nothing done
        FAILED     // no answer or CRC mismatch
    };

    uint8_t version = 0;

    AGS10(uint8_t sdaPin, uint8_t sclPin, uint32_t freqHz = 10000)
        : bus(sdaPin, sclPin, freqHz) {}

    // Initialises the bus and checks that the sensor answers.
    bool begin() {
        bus.begin();
        uint8_t d[5];
        for (int attempt = 0; attempt < 3; attempt++) {
            if (readReg(REG_VERSION, d)) {
                version = d[3];
                return true;
            }
            delay(100);
        }
        return false;
    }

    Result readTVOC(uint32_t &ppb) {
        unsigned long now = millis();
        if (acqDone && now - lastAcq < MIN_ACQ_MS) return TOO_SOON;
        lastAcq = now;
        acqDone = true;

        uint8_t d[5];
        if (!readReg(REG_TVOC, d)) return FAILED;
        if (d[0] & 0x01)           return WARMING;

        ppb = ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3];
        return VALUE;
    }

private:
    SoftI2C       bus;
    unsigned long lastCmd = 0;
    bool          cmdDone = false;
    unsigned long lastAcq = 0;
    bool          acqDone = false;

    // Keep >= 30 ms between consecutive commands (datasheet, table 2 note 1)
    void cmdGap() {
        if (!cmdDone) return;
        unsigned long dt = millis() - lastCmd;
        if (dt < CMD_GAP_MS) delay(CMD_GAP_MS - dt);
    }
    void cmdMark() { lastCmd = millis(); cmdDone = true; }

    // Write register pointer, wait, read 5 bytes, verify CRC.
    bool readReg(uint8_t reg, uint8_t out[5]) {
        cmdGap();
        bool ok = bus.write(ADDR, &reg, 1);
        cmdMark();
        if (!ok) return false;

        cmdGap();
        ok = bus.read(ADDR, out, 5);
        cmdMark();
        if (!ok) return false;

        return crc8(out, 4) == out[4];
    }

    static uint8_t crc8(const uint8_t *data, uint8_t len) {
        uint8_t crc = 0xFF;
        for (uint8_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (uint8_t b = 0; b < 8; b++)
                crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
        return crc;
    }
};
