#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// LCD address (0x27 or 0x3F usually)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// MPU6050 object
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // Init LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MPU6050 Init...");

  // Init MPU6050
  if (!mpu.begin()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MPU6050 FAIL");
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MPU6050 Ready");
  delay(1000);

  // Configure sensor ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // Get sensor event data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print accelerometer values on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("X:");
  lcd.print(a.acceleration.x, 1);
  lcd.print(" Y:");
  lcd.print(a.acceleration.y, 1);

  lcd.setCursor(0, 1);
  lcd.print("Z:");
  lcd.print(a.acceleration.z, 1);

  // Debug in Serial
  Serial.print("Accel X: "); Serial.print(a.acceleration.x);
  Serial.print(" Y: "); Serial.print(a.acceleration.y);
  Serial.print(" Z: "); Serial.println(a.acceleration.z);

  delay(500);
}
