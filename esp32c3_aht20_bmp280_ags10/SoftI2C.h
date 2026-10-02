#pragma once

#include <Arduino.h>
#include "driver/gpio.h"

// ══════════════════════════════════════════════════════════
//  Minimal bit-banged I2C master
//  ────────────────────────────────────────────────────────
//  ESP32-C3 has only one hardware I2C controller, which is
//  used by the OLED / AHT20 / BMP280 at full speed. Slow
//  devices such as the AGS10 (datasheet: SCL <= 15 kHz) get
//  their own pair of pins driven in software.
//
//  Pins are configured as open-drain with the internal
//  pull-ups enabled. At 10 kHz and short wires this is
//  enough even if the module has no pull-ups of its own.
//  Clock stretching by the slave is supported.
// ══════════════════════════════════════════════════════════

class SoftI2C {
public:
    SoftI2C(uint8_t sdaPin, uint8_t sclPin, uint32_t freqHz = 10000)
        : _sda((gpio_num_t)sdaPin),
          _scl((gpio_num_t)sclPin),
          _half(freqHz ? (500000UL / freqHz) : 50) {}

    void begin() {
        gpio_config_t c = {};
        c.pin_bit_mask = (1ULL << _sda) | (1ULL << _scl);
        c.mode         = GPIO_MODE_INPUT_OUTPUT_OD;
        c.pull_up_en   = GPIO_PULLUP_ENABLE;
        c.pull_down_en = GPIO_PULLDOWN_DISABLE;
        c.intr_type    = GPIO_INTR_DISABLE;
        gpio_config(&c);

        gpio_set_level(_sda, 1);
        gpio_set_level(_scl, 1);
        delayMicroseconds(_half * 2);
        recover();
    }

    // If a slave is holding SDA low (e.g. the MCU was reset in the
    // middle of a transfer), clock it out and issue a STOP.
    void recover() {
        for (int i = 0; i < 9 && !sdaRead(); i++) {
            sclLow();  wait();
            sclHigh(); wait();
        }
        sclLow();  wait();           // STOP must start with SCL low
        stop();
    }

    // Write `len` bytes to a device. True if everything was ACKed.
    bool write(uint8_t addr, const uint8_t *data, size_t len) {
        bool ok = start() && writeByte((uint8_t)(addr << 1));
        for (size_t i = 0; ok && i < len; i++) ok = writeByte(data[i]);
        stop();
        return ok && !_err;
    }

    // Read `len` bytes from a device. True on success.
    bool read(uint8_t addr, uint8_t *buf, size_t len) {
        bool ok = start() && writeByte((uint8_t)((addr << 1) | 1));
        for (size_t i = 0; ok && i < len && !_err; i++)
            buf[i] = readByte(i + 1 < len);   // ACK all but the last byte
        stop();
        return ok && !_err;
    }

private:
    gpio_num_t _sda, _scl;
    uint32_t   _half;          // half SCL period, µs
    bool       _err = false;   // clock-stretch timeout during a transfer

    static constexpr uint32_t STRETCH_TIMEOUT_US = 25000;

    inline void wait()    { delayMicroseconds(_half); }
    inline void sdaLow()  { gpio_set_level(_sda, 0); }
    inline void sdaHigh() { gpio_set_level(_sda, 1); }   // release
    inline bool sdaRead() { return gpio_get_level(_sda); }
    inline void sclLow()  { gpio_set_level(_scl, 0); delayMicroseconds(2); }

    // Release SCL and wait until the slave lets it go high.
    bool sclHigh() {
        gpio_set_level(_scl, 1);
        uint32_t t0 = micros();
        while (!gpio_get_level(_scl)) {
            if (micros() - t0 > STRETCH_TIMEOUT_US) { _err = true; return false; }
        }
        return true;
    }

    bool start() {
        _err = false;
        sdaHigh();
        if (!sclHigh()) return false;
        wait();
        if (!sdaRead()) {           // bus stuck — try to free it once
            recover();
            _err = false;
            sdaHigh();
            if (!sclHigh()) return false;
            wait();
            if (!sdaRead()) { _err = true; return false; }
        }
        sdaLow();  wait();
        sclLow();  wait();
        return true;
    }

    void stop() {
        sdaLow();  wait();
        sclHigh(); wait();
        sdaHigh(); wait();
    }

    bool writeByte(uint8_t b) {
        for (uint8_t m = 0x80; m; m >>= 1) {
            (b & m) ? sdaHigh() : sdaLow();
            wait();
            if (!sclHigh()) return false;
            wait();
            sclLow();
        }
        sdaHigh();                  // release SDA for the ACK bit
        wait();
        if (!sclHigh()) return false;
        wait();
        bool ack = !sdaRead();
        sclLow();
        return ack;
    }

    uint8_t readByte(bool ack) {
        uint8_t b = 0;
        sdaHigh();
        for (int i = 0; i < 8; i++) {
            wait();
            if (!sclHigh()) return 0;
            wait();
            b = (uint8_t)((b << 1) | (sdaRead() ? 1 : 0));
            sclLow();
        }
        ack ? sdaLow() : sdaHigh(); // ACK = more bytes wanted, NACK = last
        wait();
        sclHigh();
        wait();
        sclLow();
        sdaHigh();
        return b;
    }
};
