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
    limitSwitch.update();

    // If pressing the switch, move away from it
    if (limitSwitch.read() == HIGH) {
        runUntilLimitSwitch(-1, true);
    }

    // Now do the actual homing
    runUntilLimitSwitch(+1, false);
    stepper.setCurrentPosition(0);
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

    // Reset the step counter
    teachedPosition = 0;

    // Move off switch if necessary
    limitSwitch.update();
    if (limitSwitch.read() == HIGH) {
        runUntilLimitSwitch(-1, true);
    }

    // Perform homing while counting steps
    stepper.setCurrentPosition(0);
    stepper.setSpeed(homingSpeed);

    while (true) {
        limitSwitch.update();
        if (limitSwitch.read() == LOW) {
            stepper.runSpeed();
        } else {
            break;
        }
    }

    teachedPosition = stepper.currentPosition();
    stepper.setCurrentPosition(0);
}

void TeachableAxis::gotoSavedPosition() {
    if (teachedPosition == 0) {
        Serial.println("Error: No position has been taught yet");
        return;
    }
    
    // First move to saved position
    moveSteps(-teachedPosition);
    
    // Extend cylinder immediately at saved position
    digitalWrite(cylinderPin, LOW);
    
    // Wait 100ms before activating suction
    delay(100);
    
    // Activate suction
    digitalWrite(10, LOW);
    
    // Wait remaining 900ms to complete the 1 second total delay
    delay(900);
    
    // Retract cylinder and wait for it to retract
    digitalWrite(cylinderPin, HIGH);
    delay(1000);  // Wait 1 second for cylinder to retract
    
    // First return to home
    moveSteps(teachedPosition);  // Move back to home
    
    // Then move to 5 inches from home
    moveSteps(-stepsPerInch * 5);   // Move 5 inches from home
    
    // Move servo to 180 degrees
    myservo.write(180);
}

void TeachableAxis::home() {
    performHoming();
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