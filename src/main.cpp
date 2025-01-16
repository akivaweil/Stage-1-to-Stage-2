#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>
#include <Servo.h>

// TeachableAxis class definition
class TeachableAxis {
public:
    // Pin definitions consolidated
    static const int VACUUM_PIN = 12;
    static const int STAGE_2_START_SIGNAL = 6;
    static const int START_SIGNAL_PIN = 7;
    static const int STEP_PIN = 2;
    static const int DIR_PIN = 3;
    static const int ENABLE_PIN = 4;
    static const int LIMIT_SW_PIN = 5;
    static const int CYLINDER_PIN = 11;
    static const int SERVO_PIN = 9;

    // Distance parameters
    float pickupDistance = 22;  // Default pickup distance in inches
    float releaseDistance = 1.992;  // Default release distance in inches

    // Timing configuration (all values in milliseconds)
    struct Timing {
        static const int DELAY_AFTER_VACUUM_ON = 500;        // Wait after turning vacuum on
        static const int DELAY_CYLINDER_EXTEND_PICKUP = 2000; // Wait for cylinder to fully extend at pickup
        static const int DELAY_CYLINDER_EXTEND_RELEASE = 1000;// Wait for cylinder to fully extend at release
        static const int DELAY_CYLINDER_RETRACT = 1500;      // Wait for cylinder to fully retract
        static const int DELAY_BEFORE_VACUUM_OFF = 1;        // Wait before turning vacuum off at release
        static const int DELAY_AFTER_VACUUM_OFF = 500;       // Wait after turning vacuum off
        static const int DELAY_SERVO_MOVEMENT = 1;           // Wait for servo to complete movement
        static const int DELAY_BEFORE_HOMING = 1000;         // Wait before starting homing sequence
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
        bool homeDir = HIGH,
        float pickupDist = 26.657,
        float releaseDist = 2.760
    );

    void begin();
    void gotoPickupPosition();
    void home();
    void update();
    
    void setVacuum(bool on) {
        digitalWrite(VACUUM_PIN, on ? LOW : HIGH);
        Serial.print("Vacuum: ");
        Serial.println(on ? "ON" : "OFF");
    }
    
    bool getVacuumState() {
        return digitalRead(VACUUM_PIN) == LOW;
    }

    bool processSerialCommand(const String& command);
    void setPickupDistance(float distance) { 
        if (distance > 0) {
            pickupDistance = distance;
        }
    }
    void setReleaseDistance(float distance) { 
        if (distance > 0) {
            releaseDistance = distance;
        }
    }

    const float stepsPerInch;

    void moveToAbsolutePosition(long position);

private:
    const int stepPin;
    const int dirPin;
    const int enablePin;
    const int limitSwitchPin;
    const int cylinderPin;
    const int servoPin;

    const float homingSpeed;
    const float maxSpeed;
    const float acceleration;
    const bool homeDir;
    const bool moveDir;

    AccelStepper stepper;
    Bounce limitSwitch;
    Bounce startSignal;
    Servo myservo;
    bool atPickupPosition = false;

    void runUntilLimitSwitch(int moveDirection, bool runUntilHigh);
    void moveSteps(long steps);
    void performHoming();
    void executePickupSequence();
    void rotateServo(int startAngle, int endAngle);
    void extendAndRetractCylinder(bool isRelease = false);
    bool activateVacuum();
};

// TeachableAxis implementation
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
    bool homeDir,
    float pickupDist,
    float releaseDist
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
    limitSwitch(),
    startSignal()
{
    // Initialize distances after validation
    setPickupDistance(pickupDist);
    setReleaseDistance(releaseDist);
    
    // Initialize outputs
    pinMode(VACUUM_PIN, OUTPUT);
    pinMode(STAGE_2_START_SIGNAL, OUTPUT);
    digitalWrite(VACUUM_PIN, HIGH);
    digitalWrite(STAGE_2_START_SIGNAL, LOW);
}

void TeachableAxis::begin() {
    pinMode(limitSwitchPin, INPUT_PULLUP);

    limitSwitch.attach(limitSwitchPin);
    limitSwitch.interval(10); // 10 ms debounce

    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(acceleration);
    stepper.setEnablePin(enablePin);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();

    // Initialize start signal pin
    pinMode(START_SIGNAL_PIN, INPUT_PULLUP);
    startSignal.attach(START_SIGNAL_PIN);
    startSignal.interval(80); // Changed from 10ms to 50ms debounce

    myservo.attach(servoPin);
    myservo.write(80);  // Start at 80 degrees
    
    digitalWrite(cylinderPin, HIGH);
    setVacuum(false);
    delay(500);
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
    
    // If we're already at the limit switch (HOME position)
    if (limitSwitch.read() == LOW) {
        // Move away from home first
        runUntilLimitSwitch(+1, false);
        delay(500); // Small delay to ensure we're clear of the switch
    }

    // Now move towards home until we hit the limit switch
    runUntilLimitSwitch(-1, false);
    stepper.setCurrentPosition(0);
}

void TeachableAxis::moveSteps(long steps) {
    stepper.move(steps);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    if (stepper.currentPosition() != stepper.targetPosition()) {
        Serial.println("Warning: Position mismatch after move");
    }
}

void TeachableAxis::gotoPickupPosition() {
    if (!atPickupPosition) {
        executePickupSequence();
    } else {
        atPickupPosition = false;
        executePickupSequence();
    }
}

void TeachableAxis::executePickupSequence() {
    // Move to pickup position
    moveToAbsolutePosition(-stepsPerInch * pickupDistance);
    
    // Turn on vacuum and wait for it to establish
    if (!activateVacuum()) {
        Serial.println("Error: Failed to establish vacuum");
        return;
    }
    
    extendAndRetractCylinder();
    
    // Store and modify acceleration temporarily
    const float originalAccel = stepper.acceleration();
    stepper.setAcceleration(10000);
    
    // Move to release position is now handled in extendAndRetractCylinder
    stepper.setAcceleration(originalAccel);
    
    // At release position: First rotate servo to 180 and wait 1 second
    rotateServo(80, 180);
    delay(1000);  // Wait 1 second with servo at 180
    
    // Then extend cylinder
    digitalWrite(cylinderPin, LOW);
    delay(Timing::DELAY_CYLINDER_EXTEND_RELEASE);
    
    // At release position: Release vacuum
    delay(Timing::DELAY_BEFORE_VACUUM_OFF);
    setVacuum(false);
    
    // At release position: Signal stage 2
    delay(600);  // Wait after vacuum is fully released
    digitalWrite(STAGE_2_START_SIGNAL, HIGH);
    delay(600);  // Hold signal high
    digitalWrite(STAGE_2_START_SIGNAL, LOW);
    
    // At release position: Retract cylinder and return servo
    digitalWrite(cylinderPin, HIGH);
    delay(Timing::DELAY_CYLINDER_RETRACT);
    rotateServo(180, 80);
    
    // Return to pickup position
    moveToAbsolutePosition(-stepsPerInch * pickupDistance);
    
    atPickupPosition = true;
    Serial.println("Cycle complete. Send 'g' command to start next cycle.");
}

void TeachableAxis::moveToAbsolutePosition(long position) {
    stepper.moveTo(position);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    stepper.setCurrentPosition(position);
}

void TeachableAxis::rotateServo(int startAngle, int endAngle) {
    int increment = (startAngle < endAngle) ? 2 : -2;
    for(int angle = startAngle; 
        (increment > 0) ? (angle <= endAngle) : (angle >= endAngle); 
        angle += increment) {
        myservo.write(angle);
        delay(20);
    }
    delay(Timing::DELAY_SERVO_MOVEMENT);
}

void TeachableAxis::extendAndRetractCylinder(bool isRelease) {
    digitalWrite(cylinderPin, LOW);
    delay(Timing::DELAY_CYLINDER_EXTEND_PICKUP);  // Uses pickup delay since this is called at pickup position
    
    if (isRelease) {
        delay(Timing::DELAY_BEFORE_VACUUM_OFF);
        setVacuum(false);
        delay(Timing::DELAY_AFTER_VACUUM_OFF);
    }
    
    // Start retracting cylinder
    digitalWrite(cylinderPin, HIGH);
    
    // Wait 100ms to let cylinder start retracting before moving
    delay(100);
    
    // Store original speed and acceleration
    float originalSpeed = stepper.maxSpeed();
    float originalAccel = stepper.acceleration();
    
    // Temporarily reduce speed to 25% and acceleration to 50%
    stepper.setMaxSpeed(originalSpeed * 0.25);
    stepper.setAcceleration(originalAccel * 0.5);
    
    // Move directly to release position
    moveToAbsolutePosition(-stepsPerInch * releaseDistance);
    
    // Restore original speed and acceleration
    stepper.setMaxSpeed(originalSpeed);
    stepper.setAcceleration(originalAccel);
    
    // Wait remaining retraction time (subtract the 100ms we waited earlier)
    delay(Timing::DELAY_CYLINDER_RETRACT - 100);
}

void TeachableAxis::home() {
    performHoming();
}

void TeachableAxis::update() {
    limitSwitch.update();
    startSignal.update();
    
    // Check for start signal
    if (startSignal.fell()) {  // Trigger on falling edge
        gotoPickupPosition();
    }
}

bool TeachableAxis::processSerialCommand(const String& command) {
    if (command == "goto" || command == "g") {
        gotoPickupPosition();
        return true;
    }
    else if (command == "home" || command == "h") {
        digitalWrite(cylinderPin, HIGH);
        setVacuum(false);
        myservo.write(80);
        Serial.println("Moving to home position...");
        home();
        Serial.println("Home position found!");
        return true;
    }
    return false;
}

bool TeachableAxis::activateVacuum() {
    setVacuum(true);
    delay(Timing::DELAY_AFTER_VACUUM_ON);
    return getVacuumState();
}

// Create TeachableAxis object using static pins
TeachableAxis axis(
    TeachableAxis::STEP_PIN,
    TeachableAxis::DIR_PIN,
    TeachableAxis::ENABLE_PIN,
    TeachableAxis::LIMIT_SW_PIN,
    TeachableAxis::CYLINDER_PIN,
    TeachableAxis::SERVO_PIN
);

void setup() {
    Serial.begin(115200);
    pinMode(TeachableAxis::CYLINDER_PIN, OUTPUT);
    digitalWrite(TeachableAxis::CYLINDER_PIN, HIGH);
    axis.begin();
    
    Serial.println("Performing initial homing...");
    axis.home();
    Serial.println("Homing complete!");
    
    // Extend cylinder for 2 seconds and retract after homing
    Serial.println("Testing cylinder...");
    digitalWrite(TeachableAxis::CYLINDER_PIN, LOW);   // Extend
    delay(2000);                                      // Wait 2 seconds
    digitalWrite(TeachableAxis::CYLINDER_PIN, HIGH);  // Retract
    delay(1500);                                      // Wait for retraction
    
    // Move to pickup position after homing
    Serial.println("Moving to pickup position...");
    axis.moveToAbsolutePosition(-axis.stepsPerInch * axis.pickupDistance);
    Serial.println("Ready at pickup position!");
    
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
            Serial.println("Unknown command");
        }
    }
}
