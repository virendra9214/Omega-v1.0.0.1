#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BluetoothSerial.h>

// LCD I2C address (common: 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);  

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_LCD"); // Bluetooth name
  Serial.println("Bluetooth LCD Ready. Connect to 'ESP32_LCD'");

  // Initialize LCD
  lcd.init();      
  lcd.backlight(); 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting BT...");
}

void loop() {
  if (SerialBT.available()) {
    String message = SerialBT.readStringUntil('\n');
    message.trim();  // remove \r or spaces

    Serial.print("Received: ");
    Serial.println(message);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(message);  // Print received text
  }
}
