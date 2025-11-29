/**********************************************************************************************
 * 📡 LoRa Transmitter System with Sensors
 * 
 * Modules:
 * 1️⃣ LoRa Transmitter        -> Sends all sensor data wirelessly
 * 2️⃣ Temperature Sensor      -> MLX90614 object & ambient temperature
 * 3️⃣ ECG                     -> Analog ECG measurement
 * 4️⃣ MAX30102 PPG Sensor     -> Heart rate (BPM) and Oxygen saturation (SpO2)
 * 5️⃣ GSM/SIM900A             -> Sends SMS with sensor data
 * 6️⃣ RTC (DS3231)            -> Provides date & time
 * 7️⃣ LCD Display (I2C)       -> Shows live data on 20x2 screen
 * 
 * Author: Dasari Komalrao
 * Date: 29/November/2025
 **********************************************************************************************/

// ==========================================================
// =================== LIBRARIES / HEADERS ==================
// ==========================================================

// 🧩 I2C communication library for sensors & LCD
#include <Wire.h>

// 💻 LCD 20x2 library for display
#include <LiquidCrystal_I2C.h>

// 🌡️ Infrared temperature sensor MLX90614
#include <Adafruit_MLX90614.h>

// ⏰ Real Time Clock (RTC) library DS3231
#include <RTClib.h>

// 📡 Hardware Serial for GSM/SIM900A module
#include <HardwareSerial.h>

// ❤️ MAX30102 PPG sensor for heart rate & SpO2
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

// 📡 LoRa library for wireless data transmission
#include <SPI.h>
#include <LoRa.h>


// ==========================================================
// ===================== HARDWARE SETUP =====================
// ==========================================================

// 🖥️ LCD 20x2 initialization with I2C address 0x27
LiquidCrystal_I2C lcd(0x27, 20, 2);

// 🌡️ MLX90614 sensor object for object & ambient temperature
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// 📈 ECG analog input pin
#define ECG_PIN 34
int ecgValue = 0; // 🔄 Mapped ECG value 0-100

// ⏳ Real-time clock object
RTC_DS3231 rtc;

// 📞 GSM SIM900A HardwareSerial object
HardwareSerial sim900(1);
String phoneNumbers[] = { "9984798499", "9985798499", "9986798499" };

// ❤️🩸 MAX30102 PPG sensor buffers and variables
MAX30105 sensor;             
#define SAMPLES 100          
uint32_t irBuffer[SAMPLES];   
uint32_t redBuffer[SAMPLES];  
int32_t spo2;                
int8_t spo2Valid;            
int32_t heartRate;           
int8_t hrValid;              

// 📡 LoRa Pins and Frequency (for ESP32, adjust if needed)
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 2
#define LORA_BAND 915E6   // US: 915E6, EU: 868E6


// ==========================================================
// ===================== FUNCTIONS ==========================
// ==========================================================

// ------------------ Function: Send SMS via SIM900 ------------------
void sendSMS(String number, String message) {
  sim900.print("AT+CMGF=1\r");                  // 📬 Set text mode
  delay(100);                                   
  sim900.print("AT+CMGS=\"" + number + "\"\r"); // 👤 Set recipient
  delay(100);                                   
  sim900.print(message);                        // ✉️ Write message
  delay(100);                                   
  sim900.write(26);                             // ⌨️ Ctrl+Z to send
  delay(2000);                                  
}

// ------------------ Function: Send data via LoRa ------------------
void sendLoRa(String message) {
  LoRa.beginPacket();        // 📦 Start packet
  LoRa.print(message);       // ✉️ Add data to packet
  LoRa.endPacket();          // 🚀 Send packet
}


// ==========================================================
// ===================== SETUP =============================
// ==========================================================
void setup() {

  Serial.begin(115200); // 🔌 Serial monitor for debugging

  // ---------- Initialize I2C ----------
  Wire.begin(21, 22);        // 🧩 SDA=21, SCL=22 pins
  delay(50);                 // ⏳ Short delay for I2C setup

  // ---------- Initialize LCD ----------
  lcd.init();               
  lcd.backlight();          
  lcd.clear();              
  lcd.print("Initializing..."); 

  // ---------- Initialize MLX90614 ----------
  int retries = 0;            
  while (!mlx.begin()) {      
    lcd.clear();
    lcd.print("Temp Sensor Err");       
    Serial.println("MLX90614 not detected! Retrying...");
    retries++;
    if (retries > 5) {        
      lcd.print("Check wiring"); 
      while (1);              
    }
    delay(500);               
  }
  delay(100);                 

  // ---------- Initialize ECG Pin ----------
  pinMode(ECG_PIN, INPUT);    

  // ---------- Initialize RTC ----------
  if (!rtc.begin()) {         
    lcd.clear();
    lcd.print("RTC Error!");  
    Serial.println("RTC not detected!"); 
    while (1);                
  }

  // ⏳ Set RTC to compile-time date & time
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  Serial.println("RTC time set automatically"); 

  // ---------- Initialize SIM900 ----------
  sim900.begin(9600, SERIAL_8N1, 16, 17); 

  // ---------- Initialize MAX30102 ----------
  if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) { 
    lcd.clear();
    lcd.print("MAX30102 ERROR");           
    Serial.println("MAX30102 not detected!"); 
    while (1);                             
  }
  sensor.setup(0x7F, 4, 3, 100, 411, 16384);   
  sensor.setPulseAmplitudeRed(0xFF);           
  sensor.setPulseAmplitudeIR(0xFF);            
  sensor.setPulseAmplitudeGreen(0);            

  // ---------- Initialize LoRa ----------
  SPI.begin(18, 19, 23, LORA_SS); // SCK, MISO, MOSI, SS
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) {       
    lcd.clear();
    lcd.print("LoRa Init Err");     
    Serial.println("LoRa initialization failed!"); 
    while (1);                         
  }
  lcd.clear();
  lcd.print("LoRa Ready ✅");       
  delay(2000);
  lcd.clear();

  // ---------- LoRa Screen Placeholder ----------
  lcd.setCursor(0, 0);
  lcd.print("LoRa Transmitter");
  lcd.setCursor(0, 1);
  lcd.print("Sensors Ready ✅");
  delay(3000);
  lcd.clear();
}


// ==========================================================
// ===================== MAIN LOOP =========================
// ==========================================================
void loop() {

  // ========================================================
  // =================== TEMPERATURE SCREEN =================
  // ========================================================
  double objectTemp = mlx.readObjectTempC();   
  double ambientTemp = mlx.readAmbientTempC(); 

  int tries = 0;
  while (isnan(objectTemp) || isnan(ambientTemp)) {
    objectTemp = mlx.readObjectTempC();       
    ambientTemp = mlx.readAmbientTempC();
    tries++;
    if (tries > 5) break; 
    delay(200);           
  }

  lcd.setCursor(0, 0);     
  lcd.print("Ambient:");     
  lcd.print(ambientTemp, 2); 
  lcd.print((char)223);     
  lcd.print("C");

  lcd.setCursor(0, 1);      
  lcd.print("Object: ");     
  lcd.print(objectTemp, 2);  
  lcd.print((char)223);     
  lcd.print("C");
  delay(2000);               
  lcd.clear();


  // ========================================================
  // =================== ECG SCREEN =========================
  // ========================================================
  int rawECG = analogRead(ECG_PIN);            
  ecgValue = map(rawECG, 0, 4095, 0, 100);    

  lcd.setCursor(0, 0);
  lcd.print("ECG Raw: "); lcd.print(rawECG);  
  lcd.setCursor(0, 1);
  lcd.print("Mapped: "); lcd.print(ecgValue);  
  delay(2000);               
  lcd.clear();


  // ========================================================
  // =================== MAX30102 SCREEN ====================
  // ========================================================
  lcd.setCursor(0, 0);
  lcd.print("Reading..."); 
  for (int i = 0; i < SAMPLES; i++) {
    while (sensor.getIR() < 50000) { delay(5); }  
    irBuffer[i] = sensor.getIR();                 
    redBuffer[i] = sensor.getRed();               
    delay(10);                                    
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, SAMPLES,
    redBuffer,
    &spo2, &spo2Valid,
    &heartRate, &hrValid
  );

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BPM:"); lcd.print(heartRate);    
  lcd.setCursor(0, 1);
  lcd.print("SpO2:"); lcd.print(spo2);        
  lcd.print("%");
  delay(2000);               
  lcd.clear();


  // ========================================================
  // =================== RTC SCREEN =========================
  // ========================================================
  DateTime now = rtc.now();                      

  lcd.setCursor(0, 0);
  lcd.print("Date: ");                           
  lcd.print(now.day() < 10 ? "0" : ""); lcd.print(now.day());
  lcd.print("/");
  lcd.print(now.month() < 10 ? "0" : ""); lcd.print(now.month());
  lcd.print("/"); lcd.print(now.year());

  lcd.setCursor(0, 1);
  lcd.print("Time: ");
  lcd.print(now.hour() < 10 ? "0" : ""); lcd.print(now.hour());
  lcd.print(":"); lcd.print(now.minute() < 10 ? "0" : ""); lcd.print(now.minute());
  lcd.print(":"); lcd.print(now.second() < 10 ? "0" : ""); lcd.print(now.second());
  delay(2000);               
  lcd.clear();


  // ========================================================
  // =================== GSM & LoRa DATA ====================
  // ========================================================
  String msg = "Ambient:" + String(ambientTemp, 2) + "C, Object:" + String(objectTemp, 2) + "C";
  msg += ", ECG:" + String(ecgValue);
  msg += ", Date:" + String(now.day()) + "/" + String(now.month()) + "/" + String(now.year());
  msg += ", Time:" + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
  msg += ", BPM:" + String(heartRate) + ", SpO2:" + String(spo2) + "%";

  // 📤 Send via GSM
  lcd.setCursor(0, 0);
  lcd.print("Sending SMS...");  
  for (int i = 0; i < 3; i++) {
    sendSMS(phoneNumbers[i], msg); 
  }
  lcd.setCursor(0, 1);
  lcd.print("SMS Sent! ✅");
  delay(2000);               
  lcd.clear();

  // 🚀 Send same data via LoRa
  lcd.setCursor(0, 0);
  lcd.print("Sending LoRa...");  
  sendLoRa(msg);                   
  lcd.setCursor(0, 1);
  lcd.print("LoRa Sent! ✅");
  delay(2000);               
  lcd.clear();

} // END OF LOOP
