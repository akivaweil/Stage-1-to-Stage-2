#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>
#include <Servo.h>

// TeachableAxis class definition
class TeachableAxis {
public:
    // Pin definitions
    static const int VACUUM_PIN = 12;

    // Distance parameters
    float pickupDistance = 22.677;  // Default pickup distance in inches
    float releaseDistance = 1.992;  // Default release distance in inches

    // Timing configuration (all values in milliseconds)
    struct Timing {
        static const int VACUUM_ESTABLISH_DELAY = 500;    
        static const int CYLINDER_EXTEND_DELAY = 2000;     
        static const int CYLINDER_RETRACT_DELAY = 1000;    
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
    void setPickupDistance(float distance) { pickupDistance = distance; }
    void setReleaseDistance(float distance) { releaseDistance = distance; }

private:
    const int stepPin;
    const int dirPin;
    const int enablePin;
    const int limitSwitchPin;
    const int cylinderPin;
    const int servoPin;

    const float stepsPerInch;
    const float homingSpeed;
    const float maxSpeed;
    const float acceleration;
    const bool homeDir;
    const bool moveDir;

    AccelStepper stepper;
    Bounce limitSwitch;
    Servo myservo;
    bool atPickupPosition = false;

    void runUntilLimitSwitch(int moveDirection, bool runUntilHigh);
    void moveSteps(long steps);
    void performHoming();
    void executePickupSequence();
    void moveToAbsolutePosition(long position);
    void rotateServo(int startAngle, int endAngle);
    void extendAndRetractCylinder(bool isRelease = false);
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
    pickupDistance(pickupDist),
    releaseDistance(releaseDist),
    stepper(AccelStepper::DRIVER, stepPin, dirPin),
    limitSwitch()
{
    pinMode(VACUUM_PIN, OUTPUT);
    digitalWrite(VACUUM_PIN, HIGH);  // Start with vacuum off
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

    if (limitSwitch.read() == HIGH) {
        runUntilLimitSwitch(-1, true);
    }

    runUntilLimitSwitch(+1, false);
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
    long pickupPos = -stepsPerInch * pickupDistance;
    moveToAbsolutePosition(pickupPos);
    
    setVacuum(true);
    if (!getVacuumState()) {
        Serial.println("Warning: Vacuum failed to activate!");
    }
    delay(Timing::VACUUM_ESTABLISH_DELAY);
    
    extendAndRetractCylinder();
    
    long rotationPos = -stepsPerInch * 5;
    
    float originalAccel = stepper.acceleration();
    stepper.setAcceleration(10000);
    
    moveToAbsolutePosition(rotationPos);
    rotateServo(80, 180);
    
    long releasePos = -stepsPerInch * releaseDistance;
    moveToAbsolutePosition(releasePos);
    
    stepper.setAcceleration(originalAccel);
    
    extendAndRetractCylinder(true);
    
    rotateServo(180, 80);
    
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
    delay(Timing::SERVO_SETTLE_DELAY);
}

void TeachableAxis::extendAndRetractCylinder(bool isRelease) {
    digitalWrite(cylinderPin, LOW);
    delay(Timing::CYLINDER_EXTEND_DELAY);
    
    if (isRelease) {
        delay(Timing::VACUUM_PRE_RELEASE_DELAY);
        setVacuum(false);
        delay(Timing::VACUUM_RELEASE_DELAY);
    }
    
    digitalWrite(cylinderPin, HIGH);
    delay(Timing::CYLINDER_RETRACT_DELAY);
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
        setVacuum(false);
        myservo.write(80);
        Serial.println("Moving to home position...");
        home();
        Serial.println("Home position found!");
        return true;
    }
    return false;
}

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
);

void setup() {
    Serial.begin(115200);
    pinMode(CYLINDER_PIN, OUTPUT);
    digitalWrite(CYLINDER_PIN, HIGH);
    axis.begin();
    
    Serial.println("Performing initial homing...");
    axis.home();
    Serial.println("Homing complete!");
    
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
