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
    stepper(AccelStepper::DRIVER, stepPin, dirPin),
    limitSwitch()
{
    // Initialize vacuum pin
    pinMode(VACUUM_PIN, OUTPUT);
    digitalWrite(VACUUM_PIN, HIGH);  // Start with vacuum off
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
    
    // Ensure cylinder is retracted and vacuum is off at startup
    digitalWrite(cylinderPin, HIGH);
    setVacuum(false);
    delay(500); // Give components time to initialize
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
    // Optional: Verify position after move
    if (stepper.currentPosition() != stepper.targetPosition()) {
        Serial.println("Warning: Position mismatch after move");
    }
}

void TeachableAxis::gotoPickupPosition() {
    // Move to 22.677 inches
    long targetPosition = -stepsPerInch * 22.677;  // 22.677 inches from home
    moveSteps(targetPosition);
    stepper.setCurrentPosition(targetPosition);
    
    // At pickup position - activate vacuum first
    Serial.println("Activating vacuum...");
    setVacuum(true);
    if (!getVacuumState()) {
        Serial.println("Warning: Vacuum failed to activate!");
    }
    delay(Timing::VACUUM_ESTABLISH_DELAY);
    
    // Then extend cylinder
    Serial.println("Extending cylinder...");
    digitalWrite(cylinderPin, LOW);
    delay(Timing::CYLINDER_EXTEND_DELAY);
    
    // Retract cylinder and wait for it to retract
    digitalWrite(cylinderPin, HIGH);
    delay(Timing::CYLINDER_RETRACT_DELAY);
    
    // Reduce acceleration for smoother movement during servo rotation
    stepper.setAcceleration(10000);
    
    // Move to intermediate position (8 inches from final position)
    targetPosition = -stepsPerInch * 10;  // 8 + 2 = 10 inches from home
    moveSteps(targetPosition - stepper.currentPosition());
    stepper.setCurrentPosition(targetPosition);
    
    // Start servo rotation to 180 degrees gradually
    for(int angle = 80; angle <= 180; angle += 2) {
        myservo.write(angle);
        delay(20);  // 20ms delay between each 2-degree movement
    }
    
    // Move to 1.5 inches from home (changed from 2 inches)
    targetPosition = -stepsPerInch * 1.992;
    moveSteps(targetPosition - stepper.currentPosition());
    stepper.setCurrentPosition(targetPosition);
    
    // Reset acceleration to original value
    stepper.setAcceleration(acceleration);
    
    // Now extend cylinder for release
    digitalWrite(cylinderPin, LOW);
    delay(Timing::CYLINDER_EXTEND_DELAY);
    delay(Timing::VACUUM_PRE_RELEASE_DELAY);  // Wait before turning off vacuum
    setVacuum(false);
    delay(Timing::VACUUM_RELEASE_DELAY);
    
    // Retract cylinder
    digitalWrite(cylinderPin, HIGH);
    
    // Wait 1 second after retracting before returning servo to 80 degrees
    delay(1000);
    
    // Return servo to 80 degrees
    myservo.write(80);
    
    // Extra delay before homing
    delay(Timing::PRE_HOME_DELAY);
    
    // Home at the end of the cycle to reset position
    performHoming();
}

void TeachableAxis::home() {
    performHoming();
}

void TeachableAxis::update() {
    limitSwitch.update();
}

bool TeachableAxis::processSerialCommand(const String& command) {
    if (command == "goto" || command == "g") {
        gotoPickupPosition();
        return true;
    }
    else if (command == "home" || command == "h") {
        digitalWrite(cylinderPin, HIGH);
        setVacuum(false);  // Turn off vacuum
        myservo.write(80);  // Return servo to 80 degrees
        Serial.println("Moving to home position...");
        home();
        Serial.println("Home position found!");
        return true;
    }
    return false;
} 