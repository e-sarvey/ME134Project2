#include <AccelStepper.h> // Library made by Mike Mcauley for advanced control of steppers
#include <MultiStepper.h> // Class from AccelStepper for controlling multiple steppers simultaneously

// Define pins for the two steppers (change these based on wiring)
#define STEPPER1_STEP_PIN 4
#define STEPPER1_DIR_PIN 0
#define STEPPER2_STEP_PIN 12
#define STEPPER2_DIR_PIN 14

int stepsPerRevolution = 200;  // 200 steps for a standard NEMA 17 stepper motor

// Create AccelStepper objects for each motor
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// Create MultiStepper object to manage multiple steppers simultaneously
MultiStepper steppers;

void setup() {
  Serial.begin(115200); // Establish serial connection
  
  // STEPPER SETUP
  // Set max speed and acceleration for each stepper (adjust these values based on your motors)
  stepper1.setMaxSpeed(100);
  stepper1.setAcceleration(50);
  
  stepper2.setMaxSpeed(100);
  stepper2.setAcceleration(50);

  // Add the steppers to the MultiStepper manager
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
}

void loop() {
  long targetPositions[2];

  // Move stepper 1 forward 180 degrees
  stepper1.moveTo(lround(30.0 * stepsPerRevolution / 360.0));
  while (stepper1.run()) {
    // Keep running until the movement is completed
  }

  // Move stepper 1 back 180 degrees
  stepper1.moveTo(-lround(30.0 * stepsPerRevolution / 360.0));
  while (stepper1.run()) {
    // Keep running until the movement is completed
  }
  Serial.println("Stepper 1 moved back and forth");

  delay(1000); // Pause for a moment

  // Move stepper 2 forward 180 degrees
  stepper2.moveTo(lround(30.0 * stepsPerRevolution / 360.0));
  while (stepper2.run()) {
    // Keep running until the movement is completed
  }

  // Move stepper 2 back 180 degrees
  stepper2.moveTo(-lround(30.0 * stepsPerRevolution / 360.0));
  while (stepper2.run()) {
    // Keep running until the movement is completed
  }
  Serial.println("Stepper 2 moved back and forth");

  delay(1000); // Pause for a moment

  // Now move both steppers forward 180 degrees simultaneously
  targetPositions[0] = lround(30.0 * stepsPerRevolution / 360.0);  // Stepper 1
  targetPositions[1] = lround(30.0 * stepsPerRevolution / 360.0);  // Stepper 2

  // Move steppers to the target positions
  steppers.moveTo(targetPositions);

  // Use steppers.run() to manage motor movements until both reach their target
  while (steppers.run()) {
    // Keep running until the movement is completed
  }

  delay(1000); // Pause for a moment
  Serial.println("Both steppers moved forward 180 degrees");

  // Move both steppers back 180 degrees
  targetPositions[0] = -lround(30.0 * stepsPerRevolution / 360.0);  // Stepper 1
  targetPositions[1] = -lround(30.0 * stepsPerRevolution / 360.0);  // Stepper 2

  // Move steppers to the target positions
  steppers.moveTo(targetPositions);

  // Use steppers.run() to manage motor movements until both reach their target
  while (steppers.run()) {
    // Keep running until the movement is completed
  }

  Serial.println("Both steppers moved back 180 degrees");

  delay(1000); // Pause for a moment before looping
}
