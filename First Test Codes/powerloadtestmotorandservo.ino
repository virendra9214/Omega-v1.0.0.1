#include <ESP32Servo.h>

// ---------------- Motor Driver 1 ----------------
#define STBY1 13
#define PWMA1 26
#define PWMB1 16
#define AIN1_1 14
#define AIN2_1 15
#define BIN1_1 2
#define BIN2_1 5

// ---------------- Motor Driver 2 ----------------
#define STBY2 13   // Same standby pin used
#define PWMA2 27
#define AIN1_2 23
#define AIN2_2 19

// ---------------- Servos ----------------
#define SERVO1_PIN 17
#define SERVO2_PIN 18

Servo servo1;
Servo servo2;

int motorSpeed = 200;   // Motor speed (0–255)
int pos = 0;            // Servo position

void setup() {
  Serial.begin(115200);

  // Motor Driver 1 pins
  pinMode(STBY1, OUTPUT);
  pinMode(PWMA1, OUTPUT);
  pinMode(AIN1_1, OUTPUT);
  pinMode(AIN2_1, OUTPUT);
  pinMode(PWMB1, OUTPUT);
  pinMode(BIN1_1, OUTPUT);
  pinMode(BIN2_1, OUTPUT);

  // Motor Driver 2 pins
  pinMode(PWMA2, OUTPUT);
  pinMode(AIN1_2, OUTPUT);
  pinMode(AIN2_2, OUTPUT);

  // Enable motor drivers
  digitalWrite(STBY1, HIGH);
  digitalWrite(STBY2, HIGH);

  // Attach servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  // Start motors forward
  startMotors();
}

void loop() {
  // Sweep servos 0 → 180
  for (pos = 0; pos <= 180; pos += 5) {
    servo1.write(pos);
    servo2.write(pos);
    delay(30);
  }

  // Sweep servos 180 → 0
  for (pos = 180; pos >= 0; pos -= 5) {
    servo1.write(pos);
    servo2.write(pos);
    delay(30);
  }
}

// ---------------- Motor Functions ----------------
void startMotors() {
  // Motor driver 1 forward (both motors)
  digitalWrite(AIN1_1, HIGH);
  digitalWrite(AIN2_1, LOW);
  digitalWrite(BIN1_1, HIGH);
  digitalWrite(BIN2_1, LOW);
  analogWrite(PWMA1, motorSpeed);
  analogWrite(PWMB1, motorSpeed);

  // Motor driver 2 forward (only 1 motor connected on A side)
  digitalWrite(AIN1_2, HIGH);
  digitalWrite(AIN2_2, LOW);
  analogWrite(PWMA2, motorSpeed);

  Serial.println("All motors running forward");
}
