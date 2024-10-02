#include <AccelStepper.h>
#include <MultiStepper.h>

#define STEPPER1_STEP_PIN 4
#define STEPPER1_DIR_PIN 0
#define STEPPER2_STEP_PIN 12
#define STEPPER2_DIR_PIN 14

#define LIMIT_SWITCH_1_PIN 2   // Limit switch for motor 1 (to GND)
#define LIMIT_SWITCH_2_PIN 15  // Limit switch for motor 2 (to GND)

int stepsPerRevolution = 200;  // 200 steps for a standard NEMA 17 stepper motor
const unsigned long homingTimeout = 10000; // Timeout for homing sequence in milliseconds (10 seconds)

AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// Variables to track homed state
volatile bool motor1Homed = false;
volatile bool motor2Homed = false;

// Interrupt flags to handle stop and set position outside of ISR
volatile bool stopMotor1 = false;
volatile bool stopMotor2 = false;

void IRAM_ATTR limitSwitch1Pressed() {
  motor1Homed = true;  // Set homed flag
  stopMotor1 = true;   // Set flag to stop motor 1 outside ISR
}

void IRAM_ATTR limitSwitch2Pressed() {
  motor2Homed = true;  // Set homed flag
  stopMotor2 = true;   // Set flag to stop motor 2 outside ISR
}

void setup() {
  Serial.begin(115200); // Establish serial connection
  while (!Serial); // Wait for serial connection
  Serial.println();
  
  // STEPPER SETUP
  stepper1.setMaxSpeed(100.0);  // Set appropriate speed for NEMA 17
  stepper1.setAcceleration(50.0); // Set acceleration
  stepper2.setMaxSpeed(100.0);  // Set appropriate speed for NEMA 17
  stepper2.setAcceleration(50.0); // Set acceleration
  
  // Limit switch setup with interrupts
  pinMode(LIMIT_SWITCH_1_PIN, INPUT_PULLDOWN);  // Use internal pull-down resistor
  pinMode(LIMIT_SWITCH_2_PIN, INPUT_PULLDOWN);  // Use internal pull-down resistor
  
  // Attach interrupts to the limit switches
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_1_PIN), limitSwitch1Pressed, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_2_PIN), limitSwitch2Pressed, RISING);

  Serial.println("Stepper Control Ready.");
  Serial.println("Type '1' or '2' to select motor, followed by an angle (e.g., '1 30.0').");
  Serial.println("Type 'home' to home both motors using limit switches.");
}

void loop() {
  // Handle motor 1 stop and set position outside ISR
  if (stopMotor1) {
    stopMotor1 = false;  // Reset the flag
    stepper1.stop();
    stepper1.setCurrentPosition(100);  // Set homed position to 180 degrees (100 steps)
    Serial.println("Motor 1 homed to 180 degrees (100 steps).");
  }

  // Handle motor 2 stop and set position outside ISR
  if (stopMotor2) {
    stopMotor2 = false;  // Reset the flag
    stepper2.stop();
    stepper2.setCurrentPosition(100);  // Set homed position to 180 degrees (100 steps)
    Serial.println("Motor 2 homed to 180 degrees (100 steps).");
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read input until newline
    input.trim();  // Remove any trailing whitespace

    if (input == "home") {
      homeMotors();  // Call the home function
    } else {
      int motorNumber = input.charAt(0) - '0';  // Get the motor number (1 or 2)
      float angle = input.substring(2).toFloat();  // Get the angle from the string

      // Move the motor to the specified angle relative to the homed position
      if (motorNumber == 1) {
        moveMotor(stepper1, angle, 1);
      } else if (motorNumber == 2) {
        moveMotor(stepper2, angle, 2);
      } else {
        Serial.println("Invalid motor selection. Type '1' for motor 1 or '2' for motor 2.");
      }
    }
    
    delay(500);  // Add a small delay before allowing another input
    Serial.println("Ready for next command.");
  }
}

// Function to home both motors using limit switches with timeout
void homeMotors() {
  motor1Homed = false;  // Reset homing flags
  motor2Homed = false;

  Serial.println("Homing motor 1...");
  stepper1.setMaxSpeed(50.0);
  stepper1.setAcceleration(20.0);

  // Define the direction for homing motor 1
  int homeDirection1 = 1;  // Change this to 1 if you want the opposite direction
  
  // Move motor 1 towards the limit switch until triggered
  stepper1.move(homeDirection1 * 100000);  // Move a large number of steps in the chosen direction

  // Timeout logic for motor 1 homing
  unsigned long startTime = millis();
  while (!motor1Homed) {
    stepper1.run();
    if (millis() - startTime > homingTimeout) {
      Serial.println("Motor 1 homing timeout!");
      break;
    }
  }

  delay(1000);  // Small delay before homing motor 2

  Serial.println("Homing motor 2...");
  stepper2.setMaxSpeed(50.0);
  stepper2.setAcceleration(20.0);

  // Define the direction for homing motor 2
  int homeDirection2 = -1;  // Change this to -1 if you want the opposite direction
  
  // Move motor 2 towards the limit switch until triggered
  stepper2.move(homeDirection2 * 100000);  // Move a large number of steps in the chosen direction

  // Timeout logic for motor 2 homing
  startTime = millis();
  while (!motor2Homed) {
    stepper2.run();
    if (millis() - startTime > homingTimeout) {
      Serial.println("Motor 2 homing timeout!");
      break;
    }
  }

  Serial.println("Homing complete.");
}

// Function to move a motor to a specific angle
void moveMotor(AccelStepper &stepper, float angle, int motorNumber) {
  long targetSteps = lround(angle * stepsPerRevolution / 360.0);  // Convert angle to steps
  Serial.print("Moving motor ");
  Serial.print(motorNumber);
  Serial.print(" to angle: ");
  Serial.println(angle);

  stepper.moveTo(targetSteps);  // Move relative to the current position (homed at 180 degrees = 100 steps)
  while (stepper.run()) {
    // Wait until the motor reaches the target position
  }

  Serial.print("Motor ");
  Serial.print(motorNumber);
  Serial.println(" move complete.");
  delay(10);
}
