#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C address (0x27 or 0x3F usually)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Ultrasonic pins
#define TRIG_PIN 4
#define ECHO_PIN 36

void setup() {
  Serial.begin(115200);

  // LCD setup
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ultrasonic Init");

  // Ultrasonic setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  delay(1000);
  lcd.clear();
}

void loop() {
  // Send a 10us pulse to TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo time
  long duration = pulseIn(ECHO_PIN, HIGH);
  // Calculate distance in cm
  float distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  lcd.clear();
  lcd.setCursor(0, 0);

  if (distance > 0 && distance < 20) { // threshold 20 cm
    lcd.print("samne kuch hai");
  } else {
    lcd.print("samne kuch nahi hai");
  }

  delay(500);
}
