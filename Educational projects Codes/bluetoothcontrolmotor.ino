#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// Motor driver pin connections
#define STBY 13
#define PWMA 26
#define PWMB 16
#define AIN1 14
#define AIN2 15
#define BIN1 2
#define BIN2 5

// Motor speed (0–255)
int motorSpeed = 255;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Car"); // Bluetooth name
  Serial.println("Bluetooth Car Ready. Connect to 'ESP32_Car'");

  // Set motor pins as outputs
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  digitalWrite(STBY, HIGH); // Enable standby
}

void loop() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();  // remove extra spaces or \r

    if (command.equalsIgnoreCase("forward")) {
      moveForward();
    } 
    else if (command.equalsIgnoreCase("backward")) {
      moveBackward();
    } 
    else if (command.equalsIgnoreCase("left")) {
      turnLeft();
    } 
    else if (command.equalsIgnoreCase("right")) {
      turnRight();
    } 
    else if (command.equalsIgnoreCase("stop")) {
      stopMotors();
    }
  }
}

// ---------------- Motor Functions ----------------
void moveForward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, motorSpeed);
  analogWrite(PWMB, motorSpeed);
  Serial.println("Forward");
}

void moveBackward() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, motorSpeed);
  analogWrite(PWMB, motorSpeed);
  Serial.println("Backward");
}

void turnLeft() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, motorSpeed);
  analogWrite(PWMB, motorSpeed);
  Serial.println("Left");
}

void turnRight() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMA, motorSpeed);
  analogWrite(PWMB, motorSpeed);
  Serial.println("Right");
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  Serial.println("Stop");
}
