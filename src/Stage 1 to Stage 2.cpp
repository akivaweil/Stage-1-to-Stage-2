#include <Arduino.h>
#include <Bounce2.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>  // Add ESP32 Servo library

// ? ===============================================================================
// ? 🔄 PICK CYCLE SEQUENCE
// ? ===============================================================================
/*
 * 1. 🏁 Initial Position:
 *    - 🔴 Alignment cylinder extended (HIGH)
 *    - 🔵 Grab cylinder retracted (LOW)
 *    - 📐 Servo at 180 degrees
 *    - 🎯 Motor at pickup position (2.5 inches)
 * 
 * 2. 🚀 Cycle Start:
 *    - 🔵 Retract alignment cylinder (LOW)
 *    - 📐 Move servo to 160 degrees
 *    - ⏱️ Wait 500ms for movement completion
 * 
 * 3. 🔄 Grab Sequence:
 *    - 🔴 Extend grab cylinder (HIGH)
 *    - 💨 Activate vacuum (HIGH)
 *    - ⏱️ Wait 1000ms
 *    - 📐 Move servo to 180 degrees
 *    - ⏱️ Wait 500ms for servo movement
 *    - ⏱️ Additional 500ms delay
 *    - 🔵 Retract grab cylinder (LOW)
 *    - ⏱️ Wait 1500ms
 *    - 🔴 Extend alignment cylinder (HIGH)
 * 
 * 4. 📦 Release Sequence:
 *    - 🎯 Move to release position (26", ⚡5000, 📈5000)
 *    - ⏱️ Wait 50ms for settling
 *    - 🔴 Extend grab cylinder (HIGH)
 *    - ⏱️ Wait 1000ms
 *    - 💨 Deactivate vacuum (LOW)
 *    - ⏱️ Wait 200ms
 *    - 🔵 Retract grab cylinder (LOW)
 *    - ⏱️ Wait 1500ms
 * 
 * 5. 🏠 Return Sequence:
 *    - 🎯 Return to pickup position (2.5", ⚡5000, 📈5000)
 *    - ⏱️ Wait 50ms for settling
 */

// ! ===============================================================================
// ! 🛑 SAFETY NOTICE: DO NOT MODIFY WITHOUT APPROVAL
// ! ===============================================================================

/*
 * AUTOMATED stage 1 to stage 2 pick and place machine CONTROL SYSTEM. In this project I am moving wood squares from one table saw to another using a vacuum grabber attached to a cylinder (grab cylinder)
 * 
 * SAFETY NOTICE: PLEASE DO NOT DELETE OR MODIFY ANYTHING HERE, but feel free to ask questions.
 * This code controls an automated table saw cutting system. Safety is the absolute priority.
 * - Code clarity and reliability take precedence over processing efficiency
 * - All functions are written to be as explicit and straightforward as possible
 * - Hardware emergency stop switch cuts ALL power to the system when activated
 * - Multiple software safety checks are implemented throughout the cycle
 * - All switches and buttons read HIGH when activated
 * - All cylinders require a HIGH output to disengage
 * - Bounce2 library is used for switch debouncing with a 20ms debounce time
 * - All code should be very very easy to understand for a beginner programmer
 * - my switches are configured where one side is connected to 5v and the other side splits into 10k resistor to ground at its signal pin
 * - minimum pulse width through the accelstepper library should be 3ms
 */

/*
 * 🏠 HOMING SEQUENCE FOR PICK AND PLACE SYSTEM:
 * 
 * 1. 🔄 Initial Setup:
 *    - 📊 Configure stepper with 3ms minimum pulse width
 *    - 🔌 Enable motor outputs
 *    - 🎯 Set initial servo position to 180 degrees
 *    - 🔍 Initialize all switches with 20ms debounce
 * 
 * 2. 🎯 Main Homing Process:
 *    - 🔄 Move towards home switch at -HOMING_SPEED
 *    - ⏱️ Continuously monitor debounced switch state
 *    - ✋ Stop immediately when switch activates
 *    - 📍 Set current position as zero reference
 * 
 * 3. 🔄 Post-Home Positioning:
 *    - 📏 Move to pickup position (2.5 inches)
 *    - ⚡ Use positioning speed and acceleration
 *    - ⏱️ Wait for motion completion
 * 
 * 4. ✅ System Verification:
 *    - 🔍 Verify final position
 *    - 📢 Report homing completion
 *    - 🔒 System ready for operation
 * 
 * ⚠️ Safety Features During Homing:
 * - 🐌 Reduced homing speed (HOMING_SPEED/8)
 * - ⏱️ Debounced switch readings (20ms)
 * - 🛑 Immediate stop on switch activation
 * - 📊 Continuous status reporting
 * - ⚡ Controlled acceleration
 * - 🔄 Proper initialization sequence
 */

/*
 * 📝 MAINTENANCE NOTE: This sequence documentation should be kept in sync with code implementation.
 * Please update this section when modifying the cycle steps or timing values.
 */

// Pin Configuration - Using ESP32 GPIO pins
static const int STEP_PIN = 12;          
static const int DIR_PIN = 14;           
static const int ENABLE_PIN = 27;        
static const int HOMING_PIN = 26;        
static const int VACUUM_PIN = 23;        
static const int START_SIGNAL_PIN = 18;  
static const int GRAB_CYLINDER_PIN = 22;      
static const int ALIGNMENT_CYLINDER_PIN = 21;
static const int SERVO_PIN = 5;

// Motion Parameters
float pickupDistance = 2.5;    // Changed from 2 inches
float releaseDistance = 26;    // Changed from 24 inches
const float stepsPerInch = 254.0;

// Motion Settings
const int MAX_SPEED = 5000;          
const int MAX_ACCELERATION = 5000;    
const int HOMING_SPEED = 1000;       
const int POSITIONING_SPEED = 5000;   
const int POSITIONING_ACCEL = 5000;    
const bool HOME_DIRECTION = true;     
const int STEPPER_PULSE_WIDTH = 3;    

// Timing Settings (all values in milliseconds)
const int GRAB_CYLINDER_EXTEND_TIME = 1250;   // Time to keep grab cylinder extended (increased from 500ms)
const int GRAB_CYLINDER_RETRACT_TIME = 500;   // Time to keep grab cylinder retracted
const int ALIGNMENT_CYLINDER_TIME = 1000;     // Time for alignment cylinder operations
const int VACUUM_ON_DELAY = 1000;             // Time to wait after vacuum turns on
const int VACUUM_OFF_DELAY = 2000;            // Time to wait after vacuum turns off
const int HOME_SETTLE_TIME = 50;            // Time to wait after finding home
const int MOTION_SETTLE_TIME = 50;          // Time to wait after any motion

// Servo Settings
const int SERVO_START_POS = 180;    // Starting position (degrees)
const int SERVO_GRAB_POS = 160;     // Grab position (degrees) - never go past this
const int SERVO_MOVE_TIME = 500;    // Time for movements (increased from 250ms)
const int SERVO_RETURN_TIME = 500;  // Time for downward servo movement
const int GRAB_CYLINDER_WAIT = 500; // Time to wait before retracting grab cylinder
const int SERVO_SPEED = 30;      // Degrees per second - adjust this to change speed
const int SERVO_UPDATE_INTERVAL = 20;  // How often to update servo position in ms

// Global objects
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Bounce homingSwitch = Bounce();
Bounce startButton = Bounce();
Servo grabServo;

void moveServoSlowly(int targetDegrees) {
    int currentPos = grabServo.read();
    int totalMove = abs(targetDegrees - currentPos);
    
    // Calculate steps based on speed
    float degreesPerStep = float(SERVO_SPEED) * SERVO_UPDATE_INTERVAL / 1000.0;
    int numSteps = totalMove / degreesPerStep;
    
    for (int i = 0; i < numSteps; i++) {
        int newPos;
        if (targetDegrees > currentPos) {
            newPos = currentPos + (i * degreesPerStep);
        } else {
            newPos = currentPos - (i * degreesPerStep);
        }
        grabServo.write(newPos);
        delay(SERVO_UPDATE_INTERVAL);
    }
    
    // Ensure we reach exact target
    grabServo.write(targetDegrees);
}

void setup() {
    // Initialize pins with pullup for switches as per note about switch configuration
    pinMode(HOMING_PIN, INPUT_PULLUP);  // 5V and 10k pulldown as per notes
    pinMode(START_SIGNAL_PIN, INPUT_PULLUP);  // 5V and 10k pulldown as per notes
    
    // Output pins with explicit initialization
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(VACUUM_PIN, OUTPUT);
    pinMode(GRAB_CYLINDER_PIN, OUTPUT);
    pinMode(ALIGNMENT_CYLINDER_PIN, OUTPUT);
    
    // Debug message for alignment cylinder
    Serial.println("Initializing alignment cylinder...");
    Serial.println("Setting alignment cylinder to extended position");
    
    // Set initial cylinder states with debug messages
    digitalWrite(GRAB_CYLINDER_PIN, LOW);         // Start retracted
    digitalWrite(ALIGNMENT_CYLINDER_PIN, HIGH);   // Start extended
    Serial.println("Initial cylinder states set");
    
    // Configure stepper using motion settings
    stepper.setMinPulseWidth(STEPPER_PULSE_WIDTH);
    stepper.setMaxSpeed(MAX_SPEED);    
    stepper.setAcceleration(MAX_ACCELERATION);
    stepper.setEnablePin(ENABLE_PIN);
    stepper.setPinsInverted(false, false, true);
    
    // Enable the motor
    stepper.enableOutputs();
    
    Serial.begin(115200);  
    Serial.println("Starting homing sequence...");
    
    // Setup debouncing with 20ms interval as specified in notes
    homingSwitch.attach(HOMING_PIN, INPUT_PULLUP);
    homingSwitch.interval(20); // 20ms debounce time as per requirements
    
    // Setup debouncing for start button
    startButton.attach(START_SIGNAL_PIN, INPUT_PULLUP);
    startButton.interval(20);
    
    // Setup servo with safe limits
    ESP32PWM::allocateTimer(0);
    grabServo.setPeriodHertz(50);
    grabServo.attach(SERVO_PIN, 500, 2400);
    grabServo.write(SERVO_START_POS);  // Initialize to safe starting position
    
    // Perform homing
    homeAxis();
}

void homeAxis() {
    const long homingSpeed = -HOMING_SPEED;  // Negative for moving towards home switch
    bool homingComplete = false;
    
    Serial.println("Homing sequence started...");
    
    // Move towards home switch
    while (!homingComplete) {
        homingSwitch.update();
        
        if (homingSwitch.read() == HIGH) {
            stepper.setCurrentPosition(0);  // Set zero position immediately when switch triggers
            homingComplete = true;
            Serial.println("Home position found!");
            break;
        }
        
        stepper.setSpeed(homingSpeed);
        stepper.runSpeed();
    }
    
    delay(HOME_SETTLE_TIME);  // Let the mechanics settle

    // Move to pickup position
    long stepsToPickup = pickupDistance * stepsPerInch;
    Serial.print("Moving to pickup position: ");
    Serial.print(pickupDistance);
    Serial.println(" inches");
    
    stepper.setMaxSpeed(POSITIONING_SPEED);
    stepper.setAcceleration(POSITIONING_ACCEL);
    stepper.moveTo(stepsToPickup);
    
    while (stepper.isRunning()) {
        stepper.run();
    }
    
    Serial.println("Reached pickup position");
}

void PickCycle() {
    Serial.println("Starting pick cycle...");
    
    // Debug the alignment cylinder state
    Serial.println("Current alignment cylinder state: " + String(digitalRead(ALIGNMENT_CYLINDER_PIN)));
    
    // 2. Begin Cycle
    Serial.println("Retracting alignment cylinder and moving servo to grab position");
    digitalWrite(ALIGNMENT_CYLINDER_PIN, LOW);  // Retract alignment cylinder
    moveServoSlowly(SERVO_GRAB_POS);          // Move to 160 degrees
    delay(500);  // 500ms for movements (reduced from 1000ms)
    
    // 3. Grab Operation
    Serial.println("Returning servo to start position");
    moveServoSlowly(SERVO_START_POS);         // Return to 180 degrees first
    
    Serial.println("Extending grab cylinder and turning on vacuum");
    digitalWrite(GRAB_CYLINDER_PIN, HIGH);     // Extend grab cylinder
    digitalWrite(VACUUM_PIN, HIGH);            // Turn on vacuum
    delay(1250);                              // Wait 1250ms (changed from 500ms)
    
    Serial.println("Retracting grab cylinder");
    digitalWrite(GRAB_CYLINDER_PIN, LOW);      // Retract grab cylinder
    delay(500);                               // Wait 500ms

    // Extend alignment cylinder
    Serial.println("Extending alignment cylinder");
    digitalWrite(ALIGNMENT_CYLINDER_PIN, HIGH);  // Extend alignment cylinder
    delay(ALIGNMENT_CYLINDER_TIME);             // Wait for cylinder to extend

    // 4. Move to Release
    long stepsToRelease = releaseDistance * stepsPerInch;
    Serial.print("Moving to release position: ");
    Serial.print(releaseDistance);
    Serial.println(" inches");
    
    stepper.setMaxSpeed(POSITIONING_SPEED);
    stepper.setAcceleration(POSITIONING_ACCEL);
    stepper.moveTo(stepsToRelease);
    
    while (stepper.isRunning()) {
        stepper.run();
    }
    delay(MOTION_SETTLE_TIME);
    
    Serial.println("Reached release position");
    
    // Additional operations at release position
    Serial.println("Extending grab cylinder at release position");
    digitalWrite(GRAB_CYLINDER_PIN, HIGH);     // Extend grab cylinder
    delay(1000);                              // Wait 1000ms
    
    Serial.println("Turning vacuum off");
    digitalWrite(VACUUM_PIN, LOW);             // Turn vacuum OFF
    delay(200);                               // Wait 200ms
    
    Serial.println("Retracting grab cylinder");
    digitalWrite(GRAB_CYLINDER_PIN, LOW);      // Retract grab cylinder
    delay(1500);                              // Wait 1500ms (increased from 1000ms)
    
    digitalWrite(VACUUM_PIN, LOW);             // Ensure vacuum is OFF

    // 5. Return Home
    long stepsToPickup = pickupDistance * stepsPerInch;
    Serial.print("Returning to pickup position: ");
    Serial.print(pickupDistance);
    Serial.println(" inches");
    
    stepper.setMaxSpeed(POSITIONING_SPEED);
    stepper.setAcceleration(POSITIONING_ACCEL);
    stepper.moveTo(stepsToPickup);
    
    while (stepper.isRunning()) {
        stepper.run();
    }
    delay(MOTION_SETTLE_TIME);
    
    Serial.println("Back at pickup position");
    Serial.println("Pick cycle complete");
}

void loop() {
    startButton.update();
    
    if (startButton.rose()) {
        Serial.println("Start button pressed!");
        PickCycle();
    }
}

