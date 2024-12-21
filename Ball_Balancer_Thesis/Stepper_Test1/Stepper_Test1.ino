// Libraries
#include <AccelStepper.h>
#include <InverseKinematics.h>
#include <MultiStepper.h>
#include <stdint.h>
#include <math.h>

// Machine Parameters
Machine machine(2, 3.125, 1.75, 3.669291339); // (d, e, f, g) lengths of the machine

// Stepper Motor Definitions
AccelStepper stepperA(1, 2, 5); // (driver type, STEP, DIR) Driver A
AccelStepper stepperB(1, 3, 6); // (driver type, STEP, DIR) Driver B
AccelStepper stepperC(1, 4, 7); // (driver type, STEP, DIR) Driver C
MultiStepper steppers;          // Multi-stepper control instance

// Stepper Motor Variables
long pos[3];              // Target positions for each stepper motor
int ENA = 8;              // Enable pin for the drivers
double angOrig = 206.662752199; // Original angle for each leg

// Speed Control Variables
double speed[3] = {0, 0, 0};    // Current speed of stepper motors
double angToStep = 3200.0 / 360; // Angle-to-step conversion factor (steps per degree)

// Miscellaneous Variables
bool detected = false;          // Ball detection flag
long timeI;                  // Timing variable for delay

// Tilt Parameters
double hz = 4.25; // Platform height (fixed)
double nx = 0.0;  // X component of the normal vector (initially horizontal)
double ny = 0.0;  // Y component of the normal vector (initially horizontal)
double tiltSpeed = 0.01; // Speed of tilt changes (in radians per loop)

// Constant Z Value (Renamed to avoid conflict with the library)
double normalZ = 4.25; // Z component (fixed)

void setup() {
  Serial.begin(115200);

  // Configure MultiStepper
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);

  // Enable Pin Configuration
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH); // Disable drivers initially
  delay(1000);             // Allow user to reset the platform
  digitalWrite(ENA, LOW);  // Enable drivers

  // Move to Home Position
  moveTo(hz, nx, ny);            // Move platform to home position
  steppers.runSpeedToPosition(); // Block until at home position
}

void loop() {
  // Continuously tilt the platform by modifying normal vector components
  nx = 0.25 * sin(millis() * tiltSpeed / 1000.0); // Sine wave for smooth tilting on X-axis
  ny = 0.25 * cos(millis() * tiltSpeed / 1000.0); // Cosine wave for smooth tilting on Y-axis

  // Constrain nx and ny to be within the range [-0.25, 0.25]
  nx = constrain(nx, -0.25, 0.25);
  ny = constrain(ny, -0.25, 0.25);

  // Move the platform with the new normal vectors
  moveTo(hz, nx, ny);
  steppers.runSpeedToPosition(); // Block until at target position

  // Optional: Print the normal vector components for debugging
  Serial.print("nx: ");
  Serial.print(nx);
  Serial.print(" ny: ");
  Serial.print(ny);
  Serial.print(" normalZ: ");
  Serial.println(normalZ);

  delay(20); // Small delay to ensure smooth motor control
}

// Function to Move the Platform
void moveTo(double hz, double nx, double ny) {
  // Calculate Stepper Motor Positions based on the normal vectors
  for (int i = 0; i < 3; i++) {
    pos[i] = round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep);
  }

  // Set Stepper Speeds and Accelerations
  for (int i = 0; i < 3; i++) {
    stepperA.setMaxSpeed(800);
    stepperB.setMaxSpeed(800);
    stepperC.setMaxSpeed(800);
    stepperA.setAcceleration(500); // Fixed acceleration for smoother movement
    stepperB.setAcceleration(500);
    stepperC.setAcceleration(500);
  }

  // Move Steppers to Target Positions
  stepperA.moveTo(pos[0]);
  stepperB.moveTo(pos[1]);
  stepperC.moveTo(pos[2]);

  // Run Steppers Incrementally
  stepperA.run();
  stepperB.run();
  stepperC.run();
}
