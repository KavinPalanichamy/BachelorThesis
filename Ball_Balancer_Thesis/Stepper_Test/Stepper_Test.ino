//***Stepper Motor Test Code BY Aaed Musa***
//-----------------------------------------

// Libraries
#include <AccelStepper.h>
#include <MultiStepper.h>c:\Users\pkavi\OneDrive\Desktop\GIT\BachelorThesis\Ball_Balancer_Thesis\main.ino

// Stepper motors
AccelStepper stepperA(AccelStepper::DRIVER, 2, 5);  // (driver type, STEP, DIR) Driver A
AccelStepper stepperB(AccelStepper::DRIVER, 3, 6);  // (driver type, STEP, DIR) Driver B
AccelStepper stepperC(AccelStepper::DRIVER, 4, 7);  // (driver type, STEP, DIR) Driver C
MultiStepper steppers;                              // Create instance of MultiStepper

// Constants
const int ENA = 8;               // Enable pin for the drivers
const int STEPS_PER_REV = 3200;   // Steps per revolution of the stepper motor (adjust as needed)
const float DEGREE_TO_STEP = STEPS_PER_REV / 360.0; // Conversion factor from degrees to steps

// Variables
long positions[3] = {0, 0, 0};   // Target positions for each stepper motor

void setup() {
  // Set initial maximum speed value for the steppers (steps/sec)
  stepperA.setMaxSpeed(200.0);
  stepperB.setMaxSpeed(200.0);
  stepperC.setMaxSpeed(200.0);

  // Adding the steppers to the MultiStepper instance for coordinated control
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);

  // Enable pin setup
  pinMode(ENA, OUTPUT);    // Define enable pin as output
  digitalWrite(ENA, LOW);  // Enable the drivers initially

  delay(1000);  // Small delay to allow the user to reset the platform
}

void moveSteppersByDegrees(float degreesA, float degreesB, float degreesC) {
  // Convert degrees to steps for each motor
  positions[0] += (long)(degreesA * DEGREE_TO_STEP);
  positions[1] += (long)(degreesB * DEGREE_TO_STEP);
  positions[2] += (long)(degreesC * DEGREE_TO_STEP);

  // Move the motors to the new positions
  steppers.moveTo(positions);           // Set target positions for all motors
  steppers.runSpeedToPosition();        // Blocks until all steppers reach their target positions
}

void loop() {
  // Example: Move all motors 90 degrees clockwise
  moveSteppersByDegrees(60.0, 60.0, 60.0);
  delay(1000); // Wait for 1 second

  // Example: Move all motors 90 degrees counterclockwise
  moveSteppersByDegrees(-60.0, -60.0, -60.0);
  delay(1000); // Wait for 1 second
}
