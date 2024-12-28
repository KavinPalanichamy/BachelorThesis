// Forward declarations
void setup();
void loop();
void calculateIKPositions(double hz, double nx, double ny, long* positions);
void moveTo(long* positions);
void PID(double setpointX, double setpointY);

// Stewart Platform Control Code

// Libraries
#include <AccelStepper.h>
#include <InverseKinematics.h>
#include <MultiStepper.h>
#include <stdint.h>
#include <TouchScreen.h>
#include <math.h>

// Machine Parameters
Machine machine(2, 3.125, 1.75, 3.669291339);     // (d, e, f, g) lengths of the machine
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0);  // Touchscreen pins (XGND, YGND, X5V, Y5V)

// Stepper Motor Definitions
AccelStepper stepperA(1, 2, 5);  // (driver type, STEP, DIR) Driver A
AccelStepper stepperB(1, 3, 6);  // (driver type, STEP, DIR) Driver B
AccelStepper stepperC(1, 4, 7);  // (driver type, STEP, DIR) Driver C
MultiStepper steppers;           // Multi-stepper control instance

// Stepper Motor Variables
long pos[3];                     // Target positions for each stepper motor
int ENA = 8;                     // Enable pin for the drivers
double angOrig = 206.662752199;  // Original angle for each leg

// Speed Control Variables
double speed[3] = {0, 0, 0};     // Current speed of stepper motors
double speedPrev[3];             // Previous speed of stepper motors

// Touchscreen Variables
double Xoffset = 500;  // X offset for touchscreen center
double Yoffset = 500;  // Y offset for touchscreen center

// PID Control Variables
double kp = 0.1165, ki = 0.00120, kd = 0.1425;  // PID constants
double error[2] = {0, 0};                  // Current error for X and Y directions
double errorPrev[2];                       // Previous error for X and Y directions
double integr[2] = {0, 0};                 // Integral terms for X and Y
double deriv[2] = {0, 0};                  // Derivative terms for X and Y
double out[2];                             // PID output for X and Y
double rawErrorX, rawErrorY;               // Variables to store raw error values before normalization

// Miscellaneous Variables
double angToStep = 3200.0 / 360;  // Angle-to-step conversion factor (steps per degree)
bool detected = false;            // Ball detection flag
long timeI;                       // Timing variable for delay
unsigned long lastTime = 0;       // Timing for PID updates

void setup() {
  Serial.begin(115200);

  // Configure MultiStepper
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);

  // Enable Pin Configuration
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);  // Disable drivers initially
  delay(1000);              // Allow user to reset the platform
  digitalWrite(ENA, LOW);   // Enable drivers

  // Set initial speeds for homing
  stepperA.setMaxSpeed(600);
  stepperB.setMaxSpeed(600);
  stepperC.setMaxSpeed(600);
  stepperA.setAcceleration(300);
  stepperB.setAcceleration(300);
  stepperC.setAcceleration(300);

  // Calculate home position
  long homePos[3];
  calculateIKPositions(4.25, 0, 0, homePos);
  
  // Move to home position
  steppers.moveTo(homePos);
  steppers.runSpeedToPosition();  // This blocks until all steppers reach position
}

void loop() {
  PID(0, 0);  // Run PID control with setpoints (X: 0, Y: 0)
}

// Function to calculate inverse kinematics positions
void calculateIKPositions(double hz, double nx, double ny, long* positions) {
  for (int i = 0; i < 3; i++) {
    positions[i] = round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep);
  }
}

// Function to Move the Platform
void moveTo(long* positions) {
  stepperA.moveTo(positions[0]);
  stepperB.moveTo(positions[1]);
  stepperC.moveTo(positions[2]);


  // Run Steppers Incrementally
  stepperA.run();
  stepperB.run();
  stepperC.run();
}

void PID(double setpointX, double setpointY) {
  TSPoint p = ts.getPoint();  // Read touchscreen position
  long positions[3];          // Local array for positions
  static unsigned long lastUpdateTime = millis();  // Last update time
  unsigned long currentTime = millis();            // Current time
  double deltaTime = (currentTime - lastUpdateTime) / 1000.0;  // Time difference in seconds

  if (p.x != 0) {  // Ball detected
    detected = true;

    // Calculate Errors
    rawErrorX = Xoffset - p.x - setpointX;
    rawErrorY = Yoffset - p.y - setpointY;
    double errorZ = 4.25;

    // Normalize Error Vector
    double magnitudeX = 1760;
    double magnitudeY = 1520;
    double normX = rawErrorX / magnitudeX;
    double normY = rawErrorY / magnitudeY;

    // Compute error magnitude for dynamic speed adjustment
    double errorMagnitude = sqrt(rawErrorX * rawErrorX + rawErrorY * rawErrorY);

    // Compute PID Terms for X and Y
    for (int i = 0; i < 2; i++) {
      errorPrev[i] = error[i];
      error[i] = (i == 0) ? normX : normY;
      integr[i] += error[i];
      // Derivative term calculation with time normalization
      deriv[i] = (error[i] - errorPrev[i]) / deltaTime;
      deriv[i] = isnan(deriv[i]) || isinf(deriv[i]) ? 0 : deriv[i];

      out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i];
      out[i] = constrain(out[i], -0.25, 0.25);  // Constrain the output to prevent saturation
    }

    // Calculate IK positions before the timing loop
    calculateIKPositions(4.25, -out[0], -out[1], positions);

    // Dynamic speed and acceleration calculation based on error magnitude
    double normalizedError = constrain(errorMagnitude / 400.0, 0, 1);

    // Set dynamic speed and acceleration based on error magnitude
    double maxSpeed = 1200;          // Maximum speed
    double minSpeed = 900;           // Minimum speed
    double dynamicSpeed = minSpeed + normalizedError * (maxSpeed - minSpeed);

    double maxAccel = 2400;          // Maximum acceleration
    double minAccel = 1200;          // Minimum acceleration
    double dynamicAccel = minAccel + normalizedError * (maxAccel - minAccel);
    stepperA.setMaxSpeed(dynamicSpeed);
    stepperB.setMaxSpeed(dynamicSpeed);
    stepperC.setMaxSpeed(dynamicSpeed);

    stepperA.setAcceleration(dynamicAccel);
    stepperB.setAcceleration(dynamicAccel);
    stepperC.setAcceleration(dynamicAccel);

    // Pass dynamic speed and acceleration to moveTo function
    timeI = millis();
    while (millis() - timeI < 12) {
      moveTo(positions, dynamicSpeed, dynamicAccel);  // Pass positions, dynamic speed, and acceleration

      // Exit loop if all motors have finished their motion
      if (stepperA.distanceToGo() == 0 && stepperB.distanceToGo() == 0 && stepperC.distanceToGo() == 0) {
        break;
      }
    }
    Serial.println(String(p.x) + "," + String(p.y)+","+String(positions[0])+","+String(positions[1])+","+String(positions[2]));
    lastUpdateTime = currentTime;  // Update the last update time
  } else {
    // Handle Ball Not Detected
    long homePos[3];
    calculateIKPositions(4.25, 0, 0, homePos);
    moveTo(homePos, 800, 1000);  // Pass 0 error magnitude for home position and no speed/acceleration
  }
}


// Updated moveTo function to accept dynamic speed and acceleration
void moveTo(long* positions, double dynamicSpeed, double dynamicAccel) {
  // Set speed and acceleration dynamically

  // Move steppers to target positions
  stepperA.moveTo(positions[0]);
  stepperB.moveTo(positions[1]);
  stepperC.moveTo(positions[2]);

  // Run steppers incrementally
  stepperA.run();
  stepperB.run();
  stepperC.run();
}

