#pragma once

#include <Arduino.h>

#define TOUCH_PIN        4
#define MULTI_TAP_MS     400    // pause that ends a tap series
#define MAX_TAP_MS       1500   // a touch held longer than this is ignored
#define DEBOUNCE_MS      50

enum ButtonEvent {
    BTN_NONE = 0,
    BTN_SINGLE,
    BTN_TRIPLE
};

class TouchButton {
public:
    void begin() {
        pinMode(TOUCH_PIN, INPUT);
        lastState    = digitalRead(TOUCH_PIN);
        lastEdgeTime = millis();
        pressStart   = 0;
        lastTapTime  = 0;
        tapCount     = 0;
    }

    ButtonEvent update() {
        bool current = digitalRead(TOUCH_PIN);
        unsigned long now = millis();

        // Accept a level change only after the debounce time
        if (current != lastState && now - lastEdgeTime > DEBOUNCE_MS) {
            lastEdgeTime = now;
            lastState    = current;

            if (current == HIGH) {                     // finger touched
                pressStart = now;
            } else if (now - pressStart <= MAX_TAP_MS) { // finger released
                tapCount++;
                lastTapTime = now;
            }
        }

        // Tap series finished → report it
        if (tapCount > 0 && lastState == LOW && now - lastTapTime > MULTI_TAP_MS) {
            ButtonEvent evt = BTN_NONE;
            if      (tapCount >= 3) evt = BTN_TRIPLE;
            else if (tapCount == 1) evt = BTN_SINGLE;
            tapCount = 0;
            return evt;
        }

        return BTN_NONE;
    }

private:
    bool          lastState    = LOW;
    unsigned long lastEdgeTime = 0;
    unsigned long pressStart   = 0;
    unsigned long lastTapTime  = 0;
    int           tapCount     = 0;
};
