/*
  Omega Firmware (BLE Edition)
  Version: 1.0.0.2 (BLE switch-over)
  Date: 23-Aug-2025

  Changes vs SPP version:
  - Replaced BluetoothSerial (Classic SPP) with BLE GATT (Nordic UART-like service)
  - RX characteristic (Write/WriteNR) receives command lines (terminated by '\n')
  - TX characteristic (Notify) sends back responses/prints
  - 'SerialBT' calls replaced by blePrint/blePrintln/bleAvailable via a small queue

  Notes:
  - Works with Web Bluetooth and PictoBlox stage-mode BLE
  - Keep commands exactly as before (e.g., Move_motor_M1_Forward_at_42_%speed_for_2_seconds) 
  - If you send long lines, the BLE MTU splits them; this code reassembles by '\n'
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include "driver/dac.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"

// ================= BLE (Nordic UART-like) =================
// Service/Characteristic UUIDs (Nordic UART Service pattern)
#define BLE_SERVICE_UUID     "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_RX_UUID     "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // Write from Central -> ESP32
#define BLE_CHAR_TX_UUID     "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // Notify from ESP32 -> Central

static BLEServer*        g_server = nullptr;
static BLECharacteristic* g_txChar = nullptr;   // notify out
static BLECharacteristic* g_rxChar = nullptr;   // write in
static bool g_deviceConnected = false;

// Simple command queue built from BLE writes
#include <vector>
static String g_rxLineBuffer;
static std::vector<String> g_cmdQueue; // lines ready to process

// Helper: send text back to central
void bleNotify(const String &s) {
  if (!g_deviceConnected || !g_txChar) return;
  // Chunk notifications to <=180 bytes for safety
  size_t i = 0, n = s.length();
  while (i < n) {
    size_t chunk = min((size_t)180, n - i);
    g_txChar->setValue((uint8_t*)s.c_str() + i, chunk);
    g_txChar->notify();
    i += chunk;
    delay(2);
  }
}
void blePrint(const String& s){ bleNotify(s); }
void blePrintln(const String& s){ bleNotify(s + "\n"); }

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) override {
    std::string v = pChar->getValue();
    if (v.empty()) return;
    for (char c : v) {
      if (c == '\r') continue;
      if (c == '\n') {
        if (g_rxLineBuffer.length()) {
          g_cmdQueue.push_back(g_rxLineBuffer);
          g_rxLineBuffer = "";
        }
      } else {
        g_rxLineBuffer += c;
        // guard excessively long lines
        if (g_rxLineBuffer.length() > 512) {
          g_rxLineBuffer = "";
          blePrintln("Error: line too long");
        }
      }
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    g_deviceConnected = true;
    blePrintln("BLE connected");
  }
  void onDisconnect(BLEServer* s) override {
    g_deviceConnected = false;
    g_cmdQueue.clear();
    g_rxLineBuffer = "";
    BLEDevice::startAdvertising();
  }
};

// =============== Your original peripherals & pins ===============
// Ultrasonic
#define TRIG_PIN 4
#define ECHO_PIN 36 // input-only

// Servos
#define SERVO1_PIN 18
#define SERVO2_PIN 17
Servo myServo1, myServo2;

// WS2812B
#define LED_PIN 32
#define NUM_LEDS 1
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Color sensor (TCS3200/TCS230) — WARNING: S1 on TX0 and S2 on RX0 will block flashing if connected.
#define S0  33
#define S1  1   // TX0 (disconnect during upload)
#define S2  3   // RX0 (disconnect during upload)
#define S3  32
#define OUT 39  // input-only
unsigned long redPW, greenPW, bluePW; int red, green, blue;

// DHT11
#define DHTPIN 33
#define DHTTYPE DHT11

// Ext pins
#define IO33 33
#define IO32 32
#define IO39 39
#define RXPIN 3
#define TXPIN 1

// LCD I2C
LiquidCrystal_I2C lcd(0x27,16,2);

// Audio placeholders (no data provided in original code)
// const uint8_t forwardAudio[] PROGMEM = { };
// ... etc
const int forwardLength = 0, backwardLength = 0, leftLength = 0, rightLength = 0, okayLength = 0, hornLength = 0, sosLength = 0;
const int sampleRate = 28000;

// TB6612FNG pins (NOTE: Original code duplicates M1/M3 pins — verify PCB)
const int STBY = 13;
const int PWMB = 16; const int BIN1 = 2;  const int BIN2 = 5;   // M2
const int PWMA = 26; const int AIN1 = 14; const int AIN2 = 15;  // M1
const int PWMC = 26; const int CIN1 = 14; const int CIN2 = 15;  // M3 (shares M1 pins in original)

// Flags & globals (kept from original)
int angle_Servo1=0, angle_Servo2=0;
int speedm1=0,durationm1=0,speedm2=0,durationm2=0,speedm3=0,durationm3=0;
int speedf1=0,durationf1=0,speedb1=0,durationb1=0,speedr1=0,durationr1=0,speedl1=0,durationl1=0;

bool Get_value_ir1_digital=false, Get_value_ir1_analog=false, Get_value_ir2_digital=false, Get_value_ir2_analog=false;
bool Get_Ultrasonic_distance_cm=false, Get_Ultrasonic_distance_inches=false;
bool Rotate_Servo1_to=false, Rotate_Servo2_to=false;
bool Move_motor_M1_Forward_at_=false, Move_motor_M1_Backward_at_=false;
bool Move_motor_M2_Forward_at_=false, Move_motor_M2_Backward_at_=false;
bool Move_motor_M3_Forward_at_=false, Move_motor_M3_Backward_at_=false;
bool Start_Motor_function=false; bool wasConnected=false;
bool Move_Omega_Forward_at_=false, Move_Omega_Backward_at_=false, Move_Omega_Left_at_=false, Move_Omega_Right_at_=false;
bool Stop_M1=true, Stop_M2=true, Stop_M3=true;

bool Get_value_IO39_digital=false, Get_value_IO39_analog=false, Get_value_IO33_digital=false, Get_value_IO33_analog=false;
bool Get_value_IO32_digital=false, Get_value_IO32_analog=false;
bool Write_value_IO33_digital_HIGH=false, Write_value_IO33_digital_LOW=false;
bool Write_value_IO32_digital_HIGH=false, Write_value_IO32_digital_LOW=false;

// ====== helpers ======
long getUltrasonicDistanceCM();
long getUltrasonicDistanceINCHES();
int extractSpeed(String command);
int extractDuration(String command);
void rotateM1Forward(int speedPercent, int seconds);
void rotateM1Backward(int speedPercent, int seconds);
void rotateM2Forward(int speedPercent, int seconds);
void rotateM2Backward(int speedPercent, int seconds);
void rotateM3Forward(int speedPercent, int seconds);
void rotateM3Backward(int speedPercent, int seconds);
void MoveOmegaForward(int speedPercent, int seconds);
void MoveOmegaBackward(int speedPercent, int seconds);
void MoveOmegaLeft(int speedPercent, int seconds);
void MoveOmegaRight(int speedPercent, int seconds);
void setColor(uint8_t r,uint8_t g,uint8_t b);
void turnOffLED();

// Color sensor helpers
void setupColorSensorPins();
void releaseColorSensorPins();
void readColorSensor();

// ===== Command handling via BLE =====
void handleCommand(const String& cmd) {
  String c = cmd; c.trim();
  if (!c.length()) return;

  // Mirrors your original "if (SerialBT.available()) {...}" parser
  if (c.startsWith("Get_value_IR1_digital")) { Get_value_ir1_digital = true; blePrintln("IR1 digital queued"); }
  else if (c.startsWith("Get_value_IR1_analog")) { Get_value_ir1_analog = true; blePrintln("IR1 analog queued"); }
  else if (c.startsWith("Get_value_IR2_digital")) { Get_value_ir2_digital = true; blePrintln("IR2 digital queued"); }
  else if (c.startsWith("Get_value_IR2_analog")) { Get_value_ir2_analog = true; blePrintln("IR2 analog queued"); }

  else if (c.startsWith("Get_Ultrasonic_distance_cm")) { Get_Ultrasonic_distance_cm = true; blePrintln("Ultrasonic cm queued"); }
  else if (c.startsWith("Get_Ultrasonic_distance_inches")) { Get_Ultrasonic_distance_inches = true; blePrintln("Ultrasonic inches queued"); }

  else if (c.startsWith("Rotate_Servo1_to_")) { angle_Servo1 = c.substring(18).toInt(); Rotate_Servo1_to = true; blePrintln("Servo1 rotate queued"); }
  else if (c.startsWith("Rotate_Servo2_to_")) { angle_Servo2 = c.substring(18).toInt(); Rotate_Servo2_to = true; blePrintln("Servo2 rotate queued"); }

  else if (c.startsWith("Move_Omega_Forward_at_"))  { speedf1 = extractSpeed(c); durationf1 = extractDuration(c); Move_Omega_Forward_at_ = true; blePrintln("Omega forward queued"); }
  else if (c.startsWith("Move_Omega_Backward_at_")) { speedb1 = extractSpeed(c); durationb1 = extractDuration(c); Move_Omega_Backward_at_ = true; blePrintln("Omega backward queued"); }
  else if (c.startsWith("Move_Omega_Left_at_"))     { speedl1 = extractSpeed(c); durationl1 = extractDuration(c); Move_Omega_Left_at_ = true; blePrintln("Omega left queued"); }
  else if (c.startsWith("Move_Omega_Right_at_"))    { speedr1 = extractSpeed(c); durationr1 = extractDuration(c); Move_Omega_Right_at_ = true; blePrintln("Omega right queued"); }

  else if (c.startsWith("Stop_M1")) { Stop_M1 = true; blePrintln("Stop M1"); }
  else if (c.startsWith("Stop_M2")) { Stop_M2 = true; blePrintln("Stop M2"); }
  else if (c.startsWith("Stop_M3")) { Stop_M3 = true; blePrintln("Stop M3"); }

  else if (c.startsWith("Move_motor_M1_Forward_at_"))  { speedm1=extractSpeed(c); durationm1=extractDuration(c); Move_motor_M1_Forward_at_=true; blePrintln("M1 FWD queued"); }
  else if (c.startsWith("Move_motor_M1_Backward_at_")) { speedm1=extractSpeed(c); durationm1=extractDuration(c); Move_motor_M1_Backward_at_=true; blePrintln("M1 BWD queued"); }
  else if (c.startsWith("Move_motor_M2_Forward_at_"))  { speedm2=extractSpeed(c); durationm2=extractDuration(c); Move_motor_M2_Forward_at_=true; blePrintln("M2 FWD queued"); }
  else if (c.startsWith("Move_motor_M2_Backward_at_")) { speedm2=extractSpeed(c); durationm2=extractDuration(c); Move_motor_M2_Backward_at_=true; blePrintln("M2 BWD queued"); }
  else if (c.startsWith("Move_motor_M3_Forward_at_"))  { speedm3=extractSpeed(c); durationm3=extractDuration(c); Move_motor_M3_Forward_at_=true; blePrintln("M3 FWD queued"); }
  else if (c.startsWith("Move_motor_M3_Backward_at_")) { speedm3=extractSpeed(c); durationm3=extractDuration(c); Move_motor_M3_Backward_at_=true; blePrintln("M3 BWD queued"); }

  else if (c.startsWith("Start_Motor_function")) { Start_Motor_function = true; digitalWrite(STBY, HIGH); blePrintln("Motors ON"); }
  else if (c.startsWith("Turn_off_Motor_function")) { Start_Motor_function = false; digitalWrite(STBY, LOW); blePrintln("Motors OFF"); }

  else if (c.startsWith("Glow_RGB_With_R")) {
    int rIndex = c.indexOf('R') + 1;
    int gIndex = c.indexOf("_G") + 2;
    int bIndex = c.indexOf("_B") + 2;
    if (rIndex>0 && gIndex>0 && bIndex>0) {
      int rValue = c.substring(rIndex, c.indexOf("_G")).toInt();
      int gValue = c.substring(gIndex, c.indexOf("_B")).toInt();
      int bValue = c.substring(bIndex).toInt();
      setColor(rValue,gValue,bValue);
      blePrintln("LED set");
    }
  }
  else if (c.equalsIgnoreCase("Turn_Off_LED")) { turnOffLED(); blePrintln("LED off"); }

  else if (c.startsWith("Display_Text_")) {
    // Parse like original
    int startQuote = c.indexOf('\'');
    int endQuote = c.lastIndexOf('\'');
    if (startQuote!=-1 && endQuote!=-1 && startQuote!=endQuote) {
      String text = c.substring(startQuote+1, endQuote);
      int col=0,row=0;
      int colIndex=c.indexOf("coulmn_"); if(colIndex!=-1){ int colEnd=c.indexOf('_', colIndex+7); col = c.substring(colIndex+7, colEnd).toInt()-1; }
      int rowIndex=c.indexOf("raw_");    if(rowIndex!=-1){ row = c.substring(rowIndex+4).toInt()-1; }
      lcd.setCursor(col,row); lcd.print(text);
      blePrintln("LCD updated");
    }
  }

  else if (c.equalsIgnoreCase("Clear_16x2_LCD_Display")) { lcd.clear(); blePrintln("LCD cleared"); }

  else if (c.equalsIgnoreCase("Get_value_color_sensor")) {
    setupColorSensorPins(); readColorSensor(); releaseColorSensorPins();
    String out = String("R:")+red+"  G:"+green+"  B:"+blue; blePrintln(out);
  }

  else if (c.equalsIgnoreCase("Get_value_dht11")) {
    pinMode(DHTPIN, INPUT_PULLUP); delay(50);
    DHT dht(DHTPIN, DHTTYPE); dht.begin(); delay(100);
    float h=dht.readHumidity(), t=dht.readTemperature();
    if (isnan(h)||isnan(t)) blePrintln("Error: DHT11");
    else blePrintln(String("Temperature: ")+t+" *C, Humidity: "+h+" %");
    pinMode(DHTPIN, INPUT);
  }

  else if (c.startsWith("Get_value_IO39_digital")) { Get_value_IO39_digital=true; }
  else if (c.startsWith("Get_value_IO39_analog"))  { Get_value_IO39_analog=true; }
  else if (c.startsWith("Get_value_IO33_digital")) { Get_value_IO33_digital=true; }
  else if (c.startsWith("Get_value_IO33_analog"))  { Get_value_IO33_analog=true; }
  else if (c.startsWith("Get_value_IO32_digital")) { Get_value_IO32_digital=true; }
  else if (c.startsWith("Get_value_IO32_analog"))  { Get_value_IO32_analog=true; }
  else if (c.startsWith("Write_value_IO33_digital_HIGH")) { Write_value_IO33_digital_HIGH=true; }
  else if (c.startsWith("Write_value_IO33_digital_LOW"))  { Write_value_IO33_digital_LOW=true; }
  else if (c.startsWith("Write_value_IO32_digital_HIGH")) { Write_value_IO32_digital_HIGH=true; }
  else if (c.startsWith("Write_value_IO32_digital_LOW"))  { Write_value_IO32_digital_LOW=true; }
  else {
    blePrintln("Unknown cmd");
  }
}

// ===================== setup / loop =====================
void setup(){
  Serial.begin(115200);

  // I2C + LCD
  Wire.begin(21,22);
  lcd.init(); lcd.backlight();

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servos
  myServo1.attach(SERVO1_PIN, 500, 2000);
  myServo1.write(0);
  myServo2.attach(SERVO2_PIN, 500, 2000); // fixed: use SERVO2_PIN
  myServo2.write(0);

  // Motors
  pinMode(STBY, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMC, OUTPUT); pinMode(CIN1, OUTPUT); pinMode(CIN2, OUTPUT);

  // LED
  strip.begin(); strip.show();

  // Speaker DAC
  dac_output_enable(DAC_CHANNEL_1); // GPIO25

  // ===== BLE bring-up =====
  BLEDevice::init("OMEGA-001");
  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCallbacks());

  BLEService* service = g_server->createService(BLE_SERVICE_UUID);
  g_txChar = service->createCharacteristic(BLE_CHAR_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  g_txChar->addDescriptor(new BLE2902());

  g_rxChar = service->createCharacteristic(BLE_CHAR_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  g_rxChar->setCallbacks(new RxCallbacks());

  service->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06); // helps iOS
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  blePrintln("Waiting for BLE commands...");
}

void loop(){
  // Process incoming command queue built by onWrite
  if (!g_cmdQueue.empty()) {
    String cmd = g_cmdQueue.front();
    g_cmdQueue.erase(g_cmdQueue.begin());
    handleCommand(cmd);
  }

  // === Execute flagged actions (mirrors original) ===
  if (Get_value_ir1_digital){ int v=digitalRead(34); blePrintln(String(v)); Get_value_ir1_digital=false; }
  if (Get_value_ir1_analog){ int v=analogRead(34);  blePrintln(String(v)); Get_value_ir1_analog=false; }
  if (Get_value_ir2_digital){ int v=digitalRead(35); blePrintln(String(v)); Get_value_ir2_digital=false; }
  if (Get_value_ir2_analog){ int v=analogRead(35);  blePrintln(String(v)); Get_value_ir2_analog=false; }

  if (Get_Ultrasonic_distance_cm){ long d=getUltrasonicDistanceCM(); if(d==-1) blePrintln("Error! Check Connections"); else blePrintln(String("Distance: ")+d+" cm"); Get_Ultrasonic_distance_cm=false; }
  if (Get_Ultrasonic_distance_inches){ long d=getUltrasonicDistanceINCHES(); if(d==-1) blePrintln("Error! Check Connections"); else blePrintln(String("Distance: ")+d+" inches"); Get_Ultrasonic_distance_inches=false; }

  if (Rotate_Servo1_to){ if(angle_Servo1>=0 && angle_Servo1<=180){ for(int a=0;a<=angle_Servo1;a++){ myServo1.write(a); delay(20);} blePrintln("Servo1 done"); } else blePrintln("Invalid Servo1 angle"); Rotate_Servo1_to=false; }
  if (Rotate_Servo2_to){ if(angle_Servo2>=0 && angle_Servo2<=180){ for(int a=0;a<=angle_Servo2;a++){ myServo2.write(a); delay(20);} blePrintln("Servo2 done"); } else blePrintln("Invalid Servo2 angle"); Rotate_Servo2_to=false; }

  if (Move_Omega_Forward_at_){ if(speedf1>=0 && speedf1<=100 && durationf1>0) MoveOmegaForward(speedf1,durationf1); Move_Omega_Forward_at_=false; }
  if (Move_Omega_Backward_at_){ if(speedb1>=0 && speedb1<=100 && durationb1>0) MoveOmegaBackward(speedb1,durationb1); Move_Omega_Backward_at_=false; }
  if (Move_Omega_Left_at_){ if(speedl1>=0 && speedl1<=100 && durationl1>0) MoveOmegaLeft(speedl1,durationl1); Move_Omega_Left_at_=false; }
  if (Move_Omega_Right_at_){ if(speedr1>=0 && speedr1<=100 && durationr1>0) MoveOmegaRight(speedr1,durationr1); Move_Omega_Right_at_=false; }

  if (Move_motor_M1_Forward_at_){ if(speedm1>=0 && speedm1<=100 && durationm1>0) rotateM1Forward(speedm1,durationm1); Move_motor_M1_Forward_at_=false; }
  if (Move_motor_M1_Backward_at_){ if(speedm1>=0 && speedm1<=100 && durationm1>0) rotateM1Backward(speedm1,durationm1); Move_motor_M1_Backward_at_=false; }
  if (Move_motor_M2_Forward_at_){ if(speedm2>=0 && speedm2<=100 && durationm2>0) rotateM2Forward(speedm2,durationm2); Move_motor_M2_Forward_at_=false; }
  if (Move_motor_M2_Backward_at_){ if(speedm2>=0 && speedm2<=100 && durationm2>0) rotateM2Backward(speedm2,durationm2); Move_motor_M2_Backward_at_=false; }
  if (Move_motor_M3_Forward_at_){ if(speedm3>=0 && speedm3<=100 && durationm3>0) rotateM3Forward(speedm3,durationm3); Move_motor_M3_Forward_at_=false; }
  if (Move_motor_M3_Backward_at_){ if(speedm3>=0 && speedm3<=100 && durationm3>0) rotateM3Backward(speedm3,durationm3); Move_motor_M3_Backward_at_=false; }

  if (Get_value_IO39_digital){ blePrintln(String(digitalRead(39))); Get_value_IO39_digital=false; }
  if (Get_value_IO39_analog){  blePrintln(String(analogRead(39)));  Get_value_IO39_analog=false; }
  if (Get_value_IO33_digital){ blePrintln(String(digitalRead(33))); Get_value_IO33_digital=false; }
  if (Get_value_IO33_analog){  blePrintln(String(analogRead(33)));  Get_value_IO33_analog=false; }
  if (Get_value_IO32_digital){ blePrintln(String(digitalRead(32))); Get_value_IO32_digital=false; }
  if (Get_value_IO32_analog){  blePrintln(String(analogRead(32)));  Get_value_IO32_analog=false; }

  if (Write_value_IO33_digital_HIGH){ pinMode(IO33,OUTPUT); digitalWrite(IO33,HIGH); delay(100); pinMode(IO33,INPUT_PULLUP); Write_value_IO33_digital_HIGH=false; }
  if (Write_value_IO33_digital_LOW){  pinMode(IO33,OUTPUT); digitalWrite(IO33,LOW);  delay(100); pinMode(IO33,INPUT_PULLDOWN); Write_value_IO33_digital_LOW=false; }
  if (Write_value_IO32_digital_HIGH){ pinMode(IO32,OUTPUT); digitalWrite(IO32,HIGH); delay(100); pinMode(IO32,INPUT_PULLUP); Write_value_IO32_digital_HIGH=false; }
  if (Write_value_IO32_digital_LOW){  pinMode(IO32,OUTPUT); digitalWrite(IO32,LOW);  delay(100); pinMode(IO32,INPUT_PULLDOWN); Write_value_IO32_digital_LOW=false; }
}

// =================== Implementations ===================

// Ultrasonic
long getUltrasonicDistanceCM(){
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration==0) return -1;
  return (long)(duration*0.0343/2.0);
}
long getUltrasonicDistanceINCHES(){
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration==0) return -1;
  return (long)(duration*0.0135039/2.0);
}

// Extractors
int extractSpeed(String command){ int s=command.indexOf("_at_"); int e=command.indexOf("_%speed"); if(s!=-1 && e!=-1 && e>s){ return command.substring(s+4,e).toInt(); } return -1; }
int extractDuration(String command){ int i=command.lastIndexOf("_for_"); if(i!=-1){ String d=command.substring(i+5); d.replace("_seconds","" ); return d.toInt(); } return 0; }

// Motions
void MoveOmegaForward(int p, int sec){ int pwm = map(p,0,100,0,255); blePrintln(String("Moving Forward ")+p+"% for "+sec+"s");
  digitalWrite(STBY,HIGH); digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW); analogWrite(PWMA,pwm);
  digitalWrite(STBY,HIGH); digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW); analogWrite(PWMB,pwm);
  delay(sec*1000);
  analogWrite(PWMA,0); digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW);
  analogWrite(PWMB,0); digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW);
}
void MoveOmegaBackward(int p, int sec){ int pwm = map(p,0,100,0,255); blePrintln(String("Moving Backward ")+p+"% for "+sec+"s");
  digitalWrite(STBY,HIGH); digitalWrite(AIN1,LOW); digitalWrite(AIN2,HIGH); analogWrite(PWMA,pwm);
  digitalWrite(STBY,HIGH); digitalWrite(BIN1,LOW); digitalWrite(BIN2,HIGH); analogWrite(PWMB,pwm);
  delay(sec*1000);
  analogWrite(PWMA,0); digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW);
  analogWrite(PWMB,0); digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW);
}
void MoveOmegaRight(int p, int sec){ int pwm = map(p,0,100,0,255); blePrintln(String("Moving Right ")+p+"% for "+sec+"s");
  digitalWrite(STBY,HIGH); digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW); analogWrite(PWMA,pwm);
  digitalWrite(STBY,HIGH); digitalWrite(BIN1,LOW);  digitalWrite(BIN2,HIGH); analogWrite(PWMB,pwm);
  delay(sec*1000);
  analogWrite(PWMA,0); digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW);
  analogWrite(PWMB,0); digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW);
}
void MoveOmegaLeft(int p, int sec){ int pwm = map(p,0,100,0,255); blePrintln(String("Moving Left ")+p+"% for "+sec+"s");
  digitalWrite(STBY,HIGH); digitalWrite(AIN1,LOW);  digitalWrite(AIN2,HIGH); analogWrite(PWMA,pwm);
  digitalWrite(STBY,HIGH); digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW);  analogWrite(PWMB,pwm);
  delay(sec*1000);
  analogWrite(PWMA,0); digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW);
  analogWrite(PWMB,0); digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW);
}

void rotateM1Forward(int p, int sec){ int pwm=map(p,0,100,0,255); blePrintln(String("M1 FWD ")+p+"% for "+sec+"s");
  digitalWrite(STBY,HIGH); digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW); analogWrite(PWMA,pwm);
  delay(sec*1000); analogWrite(PWMA,0); digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW);
}
void rotateM1Backward(int p, int sec){ int pwm=map(p,0,100,0,255); blePrintln(String("M1 BWD ")+p+"% for "+sec+"s");
  digitalWrite(STBY,HIGH); digitalWrite(AIN1,LOW); digitalWrite(AIN2,HIGH); analogWrite(PWMA,pwm);
  delay(sec*1000); analogWrite(PWMA,0); digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW);
}
void rotateM2Forward(int p, int sec){ int pwm=map(p,0,100,0,255); blePrintln(String("M2 FWD ")+p+"% for "+sec+"s");
  digitalWrite(STBY,HIGH); digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW); analogWrite(PWMB,pwm);
  delay(sec*1000); analogWrite(PWMB,0); digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW);
}
void rotateM2Backward(int p, int sec){ int pwm=map(p,0,100,0,255); blePrintln(String("M2 BWD ")+p+"% for "+sec+"s");
  digitalWrite(STBY,HIGH); digitalWrite(BIN1,LOW); digitalWrite(BIN2,HIGH); analogWrite(PWMB,pwm);
  delay(sec*1000); analogWrite(PWMB,0); digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW);
}
void rotateM3Forward(int p, int sec){ int pwm=map(p,0,100,0,255); blePrintln(String("M3 FWD ")+p+"% for "+sec+"s");
  digitalWrite(STBY,HIGH); digitalWrite(CIN1,HIGH); digitalWrite(CIN2,LOW); analogWrite(PWMC,pwm);
  delay(sec*1000); analogWrite(PWMC,0); digitalWrite(CIN1,LOW); digitalWrite(CIN2,LOW);
}
void rotateM3Backward(int p, int sec){ int pwm=map(p,0,100,0,255); blePrintln(String("M3 BWD ")+p+"% for "+sec+"s");
  digitalWrite(STBY,HIGH); digitalWrite(CIN1,LOW); digitalWrite(CIN2,HIGH); analogWrite(PWMC,pwm);
  delay(sec*1000); analogWrite(PWMC,0); digitalWrite(CIN1,LOW); digitalWrite(CIN2,LOW);
}

// LED helpers
void setColor(uint8_t r,uint8_t g,uint8_t b){ strip.setPixelColor(0, strip.Color(r,g,b)); strip.show(); }
void turnOffLED(){ strip.clear(); strip.show(); }

// Color sensor
void setupColorSensorPins(){ pinMode(S0,OUTPUT); pinMode(S1,OUTPUT); pinMode(S2,OUTPUT); pinMode(S3,OUTPUT); pinMode(OUT,INPUT); digitalWrite(S0,HIGH); digitalWrite(S1,LOW);} // 20% scaling
void releaseColorSensorPins(){ pinMode(S0,INPUT); pinMode(S1,INPUT); pinMode(S2,INPUT); pinMode(S3,INPUT); pinMode(OUT,INPUT); }
void readColorSensor(){
  digitalWrite(S2,LOW);  digitalWrite(S3,LOW);  redPW = pulseIn(OUT, LOW);  red   = map(redPW, 25, 72, 255, 0);
  digitalWrite(S2,HIGH); digitalWrite(S3,HIGH); greenPW= pulseIn(OUT, LOW);  green = map(greenPW,30, 90, 255, 0);
  digitalWrite(S2,LOW);  digitalWrite(S3,HIGH); bluePW = pulseIn(OUT, LOW);  blue  = map(bluePW, 25, 70, 255, 0);
  red = constrain(red,0,255); green = constrain(green,0,255); blue = constrain(blue,0,255);
}
