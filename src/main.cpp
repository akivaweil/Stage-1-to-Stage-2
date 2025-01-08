#include <Arduino.h>
#include "TeachableAxis.h"

// Pin definitions
const int STEP_PIN = 2;
const int DIR_PIN = 3;
const int ENABLE_PIN = 4;
const int LIMIT_SW_PIN = 5;
const int CYLINDER_PIN = 11;
const int SERVO_PIN = 9;

// Create TeachableAxis object
TeachableAxis axis(
    STEP_PIN,
    DIR_PIN,
    ENABLE_PIN,
    LIMIT_SW_PIN,
    CYLINDER_PIN,
    SERVO_PIN
    // Optional parameters:
    // stepsPerInch (default: 254.0)
    // homingSpeed (default: 1500.0)
    // maxSpeed (default: 20000.0)
    // acceleration (default: 20000.0)
    // homeDir (default: HIGH)
);

void setup() {
    Serial.begin(115200);
    pinMode(CYLINDER_PIN, OUTPUT);
    digitalWrite(CYLINDER_PIN, HIGH);
    axis.begin();
    
    // Print available commands
    Serial.println("Available commands:");
    Serial.println("  teach  - Enter teach mode (disable motor)");
    Serial.println("  save   - Save current position and return to home");
    Serial.println("  goto   - Move to saved position and extend cylinder");
    Serial.println("  home   - Return to home position");
}

void loop() {
    axis.update();

    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (!axis.processSerialCommand(command)) {
            // Handle unknown command
            Serial.println("Unknown command");
        }
    }
}
