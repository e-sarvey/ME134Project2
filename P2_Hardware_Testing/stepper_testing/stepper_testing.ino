#include <AccelStepper.h> // Library made by Mike Mcauley for advanced control of steppers
#include <MultiStepper.h> // Class from AccelStepper for controlling multiple steppers simultaneously

// Define pins for the two steppers (change these based on wiring)
#define STEPPER1_STEP_PIN 36
#define STEPPER1_DIR_PIN 39
#define STEPPER2_STEP_PIN 34
#define STEPPER2_DIR_PIN 35

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
  stepper1.setMaxSpeed(200);
  stepper1.setAcceleration(100);
  
  stepper2.setMaxSpeed(200);
  stepper2.setAcceleration(100);

  // Add the steppers to the MultiStepper manager
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
}

void loop() {
  long targetPositions[2];

  // Move both steppers forward 45 degrees
  targetPositions[0] = lround(45.0 * stepsPerRevolution / 360.0);  // Stepper 1
  targetPositions[1] = lround(45.0 * stepsPerRevolution / 360.0);  // Stepper 2

  // Move steppers to the target positions
  steppers.moveTo(targetPositions);
  
  // Use steppers.run() to manage motor movements until both reach their target
  while (steppers.run()) {
    // Keep running until the movement is completed
  }
  
  delay(1000); // Pause for a moment

  // Move both steppers back 85 degrees
  targetPositions[0] = -lround(85.0 * stepsPerRevolution / 360.0);  // Stepper 1
  targetPositions[1] = -lround(85.0 * stepsPerRevolution / 360.0);  // Stepper 2

  // Move steppers to the target positions
  steppers.moveTo(targetPositions);
  
  // Use steppers.run() to manage motor movements until both reach their target
  while (steppers.run()) {
    // Keep running until the movement is completed
  }

  // Stop the loop after one complete cycle
  while (true) {
    // Infinite loop to stop further movement
  }
}
