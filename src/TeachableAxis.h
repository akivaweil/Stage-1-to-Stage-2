#ifndef TEACHABLE_AXIS_H
#define TEACHABLE_AXIS_H

#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>
#include <Servo.h>

class TeachableAxis {
public:
    // Pin definitions
    static const int VACUUM_PIN = 12;

    // Timing configuration (all values in milliseconds)
    struct Timing {
        static const int VACUUM_ESTABLISH_DELAY = 500;    
        static const int CYLINDER_EXTEND_DELAY = 1100;     
        static const int CYLINDER_RETRACT_DELAY = 200;    
        static const int VACUUM_RELEASE_DELAY = 500;      
        static const int SERVO_SETTLE_DELAY = 1000;       
        static const int VACUUM_PRE_RELEASE_DELAY = 400;  
        static const int PRE_HOME_DELAY = 1000;          
    };

    // Constructor
    TeachableAxis(
        int stepPin,
        int dirPin,
        int enablePin,
        int limitSwitchPin,
        int cylinderPin,
        int servoPin,
        float stepsPerInch = 254.0,
        float homingSpeed = 1000.0,
        float maxSpeed = 10000.0,
        float acceleration = 10000.0,
        bool homeDir = HIGH
    );

    // Initialize the axis
    void begin();

    // Main commands
    void gotoPickupPosition();
    void home();

    // Call this in your main loop
    void update();

    // Vacuum control
    void setVacuum(bool on) {
        digitalWrite(VACUUM_PIN, on ? LOW : HIGH);
        Serial.print("Vacuum: ");
        Serial.println(on ? "ON" : "OFF");
    }
    
    bool getVacuumState() {
        return digitalRead(VACUUM_PIN) == LOW;
    }

    // Process any serial commands (returns true if command was handled)
    bool processSerialCommand(const String& command);

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