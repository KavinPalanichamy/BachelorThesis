#include <TouchScreen.h>

// Touch screen pins and configuration
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0);  // (XGND, YGND, X5V, Y5V)

void setup() {
  Serial.begin(115200);  // Start serial communication
}

void loop() {
  TSPoint p = ts.getPoint();  // Get the current touch point

  // Check if the screen is being touched
  if (p.x != 0 && p.y != 0) {
    Serial.print("X: ");
    Serial.print(p.x);
    Serial.print("  Y: ");
    Serial.println(p.y);
  } else {
    Serial.println("No touch detected");
  }

  delay(100);  // Small delay to prevent flooding the serial monitor
}
