#include <Arduino.h>
#include "Streaming.h"

#include <unity.h>

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(100);
    
    UNITY_BEGIN();

    Serial.begin(500000);
    while (!Serial) {
        delay(1);
    }
    Serial.println("Setup");

    UNITY_END();
}

void loop() {
}