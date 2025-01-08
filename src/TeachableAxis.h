#ifndef TEACHABLE_AXIS_H
#define TEACHABLE_AXIS_H

#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>
#include <Servo.h>

class TeachableAxis {
public:
    // Constructor
    TeachableAxis(
        int stepPin,
        int dirPin,
        int enablePin,
        int limitSwitchPin,
        int cylinderPin,
        int servoPin,
        float stepsPerInch = 254.0,
        float homingSpeed = 1500.0,
        float maxSpeed = 10000.0,
        float acceleration = 10000.0,
        bool homeDir = HIGH
    );

    // Initialize the axis
    void begin();

    // Main commands
    void enterTeachMode();
    void savePosition();
    void gotoSavedPosition();
    void home();

    // Status checks
    bool isInTeachMode() const { return isTeachMode; }
    float getSavedPositionInches() const { return teachedPosition / stepsPerInch; }
    long getSavedPositionSteps() const { return teachedPosition; }

    // Process any serial commands (returns true if command was handled)
    bool processSerialCommand(const String& command);

    // Call this in your main loop
    void update();

    // Add these methods to the class declaration
    void setVacuum(bool on) {
        digitalWrite(10, on ? LOW : HIGH);
        Serial.print("Vacuum: ");
        Serial.println(on ? "ON" : "OFF");
    }
    
    bool getVacuumState() {
        return digitalRead(10) == LOW;
    }

private:
    // Hardware pins
    const int stepPin;
    const int dirPin;
    const int enablePin;
    const int limitSwitchPin;
    const int cylinderPin;
    const int servoPin;

    // Configuration
    const float stepsPerInch;
    const float homingSpeed;
    const float maxSpeed;
    const float acceleration;
    const bool homeDir;
    const bool moveDir;

    // State variables
    long teachedPosition;
    bool isTeachMode;

    // Objects
    AccelStepper stepper;
    Bounce limitSwitch;
    Servo myservo;

    // Helper functions
    void runUntilLimitSwitch(int moveDirection, bool runUntilHigh);
    void moveSteps(long steps);
    void performHoming();
};

#endif 