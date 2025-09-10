#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// IR sensor pins
#define IR1_PIN 34
#define IR2_PIN 35

void setup() {
  Serial.begin(115200);

  // LCD setup
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // IR pins
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
}

void loop() {
  int ir1State = digitalRead(IR1_PIN);
  int ir2State = digitalRead(IR2_PIN);

  // Print to Serial
  Serial.print("IR1: ");
  Serial.print(ir1State == HIGH ? "HIGH" : "LOW");
  Serial.print(" | IR2: ");
  Serial.println(ir2State == HIGH ? "HIGH" : "LOW");

  // Print to LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IR1: ");
  lcd.print(ir1State == HIGH ? "HIGH" : "LOW");

  lcd.setCursor(0, 1);
  lcd.print("IR2: ");
  lcd.print(ir2State == HIGH ? "HIGH" : "LOW");

  delay(500);
}
