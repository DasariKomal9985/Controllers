// ==========================================================
// =================== LIBRARIES / HEADERS ==================
// ==========================================================

// 🧩 I2C communication library
#include <Wire.h>

// 💻 LCD 20x2 library for display
#include <LiquidCrystal_I2C.h>

// 🌡️ Infrared temperature sensor MLX90614
#include <Adafruit_MLX90614.h>

// ⏰ Real Time Clock (RTC) library
#include <RTClib.h>

// 📡 Hardware Serial for GSM/SIM900A module
#include <HardwareSerial.h>

// ❤️ MAX30102 PPG sensor for heart rate & SpO2
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"


// ==========================================================
// ===================== HARDWARE SETUP =====================
// ==========================================================

// 🖥️ LCD 20x2 initialization with I2C address 0x27
LiquidCrystal_I2C lcd(0x27, 20, 2);

// 🌡️ MLX90614 sensor object for object & ambient temperature
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// 📈 ECG analog input pin
#define ECG_PIN 34
int ecgValue = 0; // 🔄 Mapped ECG value from 0-100

// ⏳ Real-time clock object
RTC_DS3231 rtc;

// 📞 GSM SIM900A HardwareSerial object
HardwareSerial sim900(1);
String phoneNumbers[] = { "9984798499", "9985798499", "9986798499" };

// ❤️🩸 MAX30102 PPG sensor buffers and variables
MAX30105 sensor;             // Sensor object
#define SAMPLES 100          // Number of samples to collect
uint32_t irBuffer[SAMPLES];  // IR LED readings buffer
uint32_t redBuffer[SAMPLES]; // RED LED readings buffer
int32_t spo2;                // Oxygen saturation %
int8_t spo2Valid;            // Validity flag for SpO2 ✅
int32_t heartRate;           // Heart rate BPM 💓
int8_t hrValid;              // Validity flag for HR ✅


// ==========================================================
// ===================== FUNCTIONS ==========================
// ==========================================================

// ------------------ Function: Send SMS via SIM900 ------------------
// ✉️ Send message to a specific number using GSM module
void sendSMS(String number, String message) {
  sim900.print("AT+CMGF=1\r");                  // 📬 Set text mode
  delay(100);                                   // ⏳ Wait for command processing
  sim900.print("AT+CMGS=\"" + number + "\"\r"); // 👤 Set recipient
  delay(100);                                   // ⏳ Wait
  sim900.print(message);                        // ✉️ Write message
  delay(100);                                   // ⏳ Wait
  sim900.write(26);                             // ⌨️ Ctrl+Z to send SMS
  delay(2000);                                  // ⏳ Wait for GSM module to send
}


// ==========================================================
// ===================== SETUP =============================
// ==========================================================
void setup() {

  Serial.begin(115200);      // 🔌 Start serial monitor for debugging

  // ---------- Initialize I2C ----------
  Wire.begin(21, 22);        // 🧩 SDA=21, SCL=22 pins for ESP32
  delay(50);                 // ⏳ Short delay for I2C setup

  // ---------- Initialize LCD ----------
  lcd.init();                // 🖥️ Initialize LCD screen
  lcd.backlight();           // 💡 Turn on LCD backlight
  lcd.clear();               // 🧹 Clear previous content
  lcd.print("Initializing..."); // 🏁 Display startup message

  // ---------- Initialize MLX90614 ----------
  int retries = 0;            // 🔄 Retry counter for sensor init
  while (!mlx.begin()) {      // ❌ Retry until sensor detected
    lcd.clear();
    lcd.print("Temp Sensor Err");       // ⚠️ Show error on LCD
    Serial.println("MLX90614 not detected! Retrying..."); // Serial debug
    retries++;
    if (retries > 5) {        // 🚨 Stop after 5 retries
      lcd.print("Check wiring"); 
      while (1);              // 🔒 Halt program
    }
    delay(500);               // ⏳ Wait before retrying
  }
  delay(100);                 // ⏳ Short delay after init

  // ---------- Initialize ECG Pin ----------
  pinMode(ECG_PIN, INPUT);    // 📈 Set ECG analog pin as input

  // ---------- Initialize RTC ----------
  if (!rtc.begin()) {         // ❌ Check if RTC is detected
    lcd.clear();
    lcd.print("RTC Error!");  // ⚠️ LCD error
    Serial.println("RTC not detected!"); // Serial debug
    while (1);                // 🔒 Stop program
  }

  // ⏳ Set RTC to compile-time date & time (first upload only)
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("RTC time set automatically"); // Serial debug

  // ---------- Initialize SIM900 ----------
  sim900.begin(9600, SERIAL_8N1, 16, 17); // 📡 UART pins for GSM module

  // ---------- Initialize MAX30102 ----------
  if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) { // ❌ Check sensor
    lcd.clear();
    lcd.print("MAX30102 ERROR");           // ⚠️ Display error
    Serial.println("MAX30102 not detected!"); // Serial debug
    while (1);                             // 🔒 Stop program
  }
  sensor.setup(0x7F, 4, 3, 100, 411, 16384);   // ⚙️ Sensor configuration
  sensor.setPulseAmplitudeRed(0xFF);           // ❤️ Red LED full brightness
  sensor.setPulseAmplitudeIR(0xFF);            // 🌈 IR LED full brightness
  sensor.setPulseAmplitudeGreen(0);            // 💚 Green LED off

  // ---------- LoRa Screen Placeholder ----------
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LoRa Transmitter"); // 📡 Placeholder message
  lcd.setCursor(0, 1);
  lcd.print("will be heading");
  delay(5000);                    // ⏳ Wait 5 seconds
  lcd.clear();
}


// ==========================================================
// ===================== MAIN LOOP =========================
// ==========================================================
void loop() {

  // ========================================================
  // =================== TEMPERATURE SCREEN =================
  // ========================================================
  double objectTemp = mlx.readObjectTempC();   // 🌡️ Read object temp
  double ambientTemp = mlx.readAmbientTempC(); // 🌡️ Read ambient temp

  // 🔁 Retry if reading is NaN
  int tries = 0;
  while (isnan(objectTemp) || isnan(ambientTemp)) {
    objectTemp = mlx.readObjectTempC();       
    ambientTemp = mlx.readAmbientTempC();
    tries++;
    if (tries > 5) break; // ❌ Stop retry after 5 attempts
    delay(200);           // ⏳ Wait before next reading
  }

  lcd.setCursor(0, 0);     
  lcd.print("Ambient:");     
  lcd.print(ambientTemp, 2); 
  lcd.print((char)223);      // ℃ symbol
  lcd.print("C");

  lcd.setCursor(0, 1);      
  lcd.print("Object: ");     
  lcd.print(objectTemp, 2);  
  lcd.print((char)223);      // ℃ symbol
  lcd.print("C");
  delay(3000);               
  lcd.clear();

  // ========================================================
  // =================== ECG SCREEN =========================
  // ========================================================
  int rawECG = analogRead(ECG_PIN);             // 📈 Read raw ECG value
  ecgValue = map(rawECG, 0, 4095, 0, 100);     // 🔄 Map to 0-100 scale

  lcd.setCursor(0, 0);
  lcd.print("ECG Raw: "); lcd.print(rawECG);  
  lcd.setCursor(0, 1);
  lcd.print("Mapped: "); lcd.print(ecgValue);  
  delay(3000);               
  lcd.clear();

  // ========================================================
  // =================== MAX30102 SCREEN ====================
  // ========================================================
  lcd.setCursor(0, 0);
  lcd.print("Reading..."); // ⏳ Measuring BPM & SpO2
  for (int i = 0; i < SAMPLES; i++) {
    while (sensor.getIR() < 50000) { delay(5); }  // 👆 Wait for finger placement
    irBuffer[i] = sensor.getIR();                 // 🌈 Store IR value
    redBuffer[i] = sensor.getRed();               // ❤️ Store RED value
    delay(10);                                    // ⏳ Small delay between readings
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, SAMPLES,
    redBuffer,
    &spo2, &spo2Valid,
    &heartRate, &hrValid
  );

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BPM:"); lcd.print(heartRate);    // ❤️ Heart Rate
  lcd.setCursor(0, 1);
  lcd.print("SpO2:"); lcd.print(spo2);        // 💨 Oxygen Saturation
  lcd.print("%");
  delay(3000);               
  lcd.clear();

  // ========================================================
  // =================== RTC SCREEN =========================
  // ========================================================
  DateTime now = rtc.now();                      // ⏰ Get current RTC time
  lcd.setCursor(0, 0);
  lcd.print("Date: ");                           
  lcd.print(now.day() < 10 ? "0" : ""); lcd.print(now.day());
  lcd.print("/");
  lcd.print(now.month() < 10 ? "0" : ""); lcd.print(now.month());
  lcd.print("/");
  lcd.print(now.year());

  lcd.setCursor(0, 1);
  lcd.print("Time: ");
  lcd.print(now.hour() < 10 ? "0" : ""); lcd.print(now.hour());
  lcd.print(":");
  lcd.print(now.minute() < 10 ? "0" : ""); lcd.print(now.minute());
  lcd.print(":");
  lcd.print(now.second() < 10 ? "0" : ""); lcd.print(now.second());
  delay(3000);               
  lcd.clear();

  // ========================================================
  // =================== GSM SCREEN =========================
  // ========================================================
  String msg = "Ambient:" + String(ambientTemp, 2) + "C, Object:" + String(objectTemp, 2) + "C";
  msg += ", ECG:" + String(ecgValue);
  msg += ", Date:" + String(now.day()) + "/" + String(now.month()) + "/" + String(now.year());
  msg += ", Time:" + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
  msg += ", BPM:" + String(heartRate) + ", SpO2:" + String(spo2) + "%";

  lcd.setCursor(0, 0);
  lcd.print("Sending Data...");  // 📤 Sending SMS notification
  for (int i = 0; i < 3; i++) {
    sendSMS(phoneNumbers[i], msg); // ✉️ Send SMS to all numbers
  }
  lcd.setCursor(0, 1);
  lcd.print("SMS Sent! ✅");       // ✅ Confirmation
  delay(3000);                     
  lcd.clear();

} // END OF LOOP
