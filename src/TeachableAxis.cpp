#include "TeachableAxis.h"

TeachableAxis::TeachableAxis(
    int stepPin,
    int dirPin,
    int enablePin,
    int limitSwitchPin,
    int cylinderPin,
    int servoPin,
    float stepsPerInch,
    float homingSpeed,
    float maxSpeed,
    float acceleration,
    bool homeDir
) : 
    stepPin(stepPin),
    dirPin(dirPin),
    enablePin(enablePin),
    limitSwitchPin(limitSwitchPin),
    cylinderPin(cylinderPin),
    servoPin(servoPin),
    stepsPerInch(stepsPerInch),
    homingSpeed(homingSpeed),
    maxSpeed(maxSpeed),
    acceleration(acceleration),
    homeDir(homeDir),
    moveDir(!homeDir),
    teachedPosition(0),
    isTeachMode(false),
    stepper(AccelStepper::DRIVER, stepPin, dirPin),
    limitSwitch()
{
    // Initialize suction pin
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH);  // Start with suction off
}

void TeachableAxis::begin() {
    // Pin modes
    pinMode(limitSwitchPin, INPUT_PULLUP);

    // Setup bounce
    limitSwitch.attach(limitSwitchPin);
    limitSwitch.interval(10); // 10 ms debounce

    // Configure AccelStepper
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(acceleration);
    stepper.setEnablePin(enablePin);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();

    // Initialize servo
    myservo.attach(servoPin);
    myservo.write(80);  // Start at 80 degrees
}

void TeachableAxis::runUntilLimitSwitch(int moveDirection, bool runUntilHigh) {
    float speed = (moveDirection > 0) ? homingSpeed : -homingSpeed;
    stepper.setSpeed(speed);

    while (true) {
        limitSwitch.update();
        if (runUntilHigh) {
            if (limitSwitch.read() == HIGH) {
                stepper.runSpeed();
            } else {
                break;
            }
        } else {
            if (limitSwitch.read() == LOW) {
                stepper.runSpeed();
            } else {
                break;
            }
        }
    }
}

void TeachableAxis::performHoming() {
    // First move away from switch if we're on it
    limitSwitch.update();
    if (limitSwitch.read() == HIGH) {
        runUntilLimitSwitch(-1, true);
        delay(500); // Small delay to ensure we're clear
    }

    // Now do the actual homing
    runUntilLimitSwitch(+1, false);
    
    // Set this as absolute zero position
    stepper.setCurrentPosition(0);
    
    // Move slightly off the switch to prevent false triggers
    moveSteps(-100); // Back off 100 steps from limit switch
}

void TeachableAxis::moveSteps(long steps) {
    stepper.move(steps);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
}

void TeachableAxis::enterTeachMode() {
    isTeachMode = true;
    stepper.disableOutputs();
}

void TeachableAxis::savePosition() {
    if (!isTeachMode) {
        Serial.println("Error: Must be in teach mode to save position");
        return;
    }

    isTeachMode = false;
    stepper.enableOutputs();

    // Remember current position before homing
    long currentPos = stepper.currentPosition();
    
    // Perform homing to establish reference
    performHoming();
    
    // Now move back to where we were and record that position
    moveSteps(-currentPos);
    teachedPosition = stepper.currentPosition();
    
    Serial.print("Position saved at: ");
    Serial.print(teachedPosition);
    Serial.println(" steps from home");
}

void TeachableAxis::gotoSavedPosition() {
    if (teachedPosition == 0) {
        Serial.println("Error: No position has been taught yet");
        return;
    }
    
    // First ensure we're at home position
    performHoming();
    
    // Move to pickup position
    moveSteps(teachedPosition);
    
    // Extend cylinder and activate suction
    digitalWrite(cylinderPin, LOW);
    delay(1000);
    digitalWrite(10, LOW); // Activate suction
    
    // Retract cylinder
    digitalWrite(cylinderPin, HIGH);
    delay(1000);
    
    // Return to home
    performHoming();
    
    // Move to drop-off position (5 inches from home)
    moveSteps(-stepsPerInch * 5);
    
    // Rotate servo for drop-off
    myservo.write(180);
    
    // Extend cylinder for drop-off
    digitalWrite(cylinderPin, LOW);
    delay(1000);
    
    // Release vacuum
    digitalWrite(10, HIGH);
    delay(500);
    
    // Retract cylinder
    digitalWrite(cylinderPin, HIGH);
}

void TeachableAxis::home() {
    performHoming();
    
    // Ensure everything is in default state
    digitalWrite(cylinderPin, HIGH);
    digitalWrite(10, HIGH);  // Suction off
    myservo.write(80);      // Default servo position
}

void TeachableAxis::update() {
    limitSwitch.update();
}

bool TeachableAxis::processSerialCommand(const String& command) {
    if (command == "teach" || command == "t") {
        digitalWrite(cylinderPin, HIGH);
        digitalWrite(10, HIGH);  // Turn off suction
        myservo.write(80);  // Return servo to 80 degrees
        enterTeachMode();
        Serial.println("Teach mode: Move gantry to desired position manually");
        Serial.println("Type 's' when position is set");
        return true;
    }
    else if (command == "save" || command == "s") {
        savePosition();
        Serial.print("Position saved: ");
        Serial.print(teachedPosition);
        Serial.print(" steps (");
        Serial.print(teachedPosition / stepsPerInch, 3);
        Serial.println(" inches) from home");
        return true;
    }
    else if (command == "goto" || command == "g") {
        gotoSavedPosition();
        return true;
    }
    else if (command == "home" || command == "h") {
        digitalWrite(cylinderPin, HIGH);
        digitalWrite(10, HIGH);  // Turn off suction
        myservo.write(80);  // Return servo to 80 degrees
        Serial.println("Moving to home position...");
        home();
        Serial.println("Home position found!");
        return true;
    }
    return false;
} 