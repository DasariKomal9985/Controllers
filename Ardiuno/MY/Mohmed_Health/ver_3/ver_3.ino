#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MLX90614.h>
#include <RTClib.h>
#include <HardwareSerial.h> // For ESP32

// LCD 20x2
LiquidCrystal_I2C lcd(0x27, 20, 2);

// MLX90614 sensor
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// ECG pin
#define ECG_PIN 34
int ecgValue = 0;

// RTC
RTC_DS3231 rtc;

// GSM SIM900A
HardwareSerial sim900(1); // Use UART1 (pins TX=17, RX=16 for example)
String phoneNumbers[] = {"9984798499","9985798499","9986798499"};

void setup() {
  Serial.begin(115200);
  sim900.begin(9600, SERIAL_8N1, 16, 17); // RX, TX pins for ESP32

  lcd.init();
  lcd.backlight();
  lcd.clear();

  if (!mlx.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("Temp Sensor Error");
    while(1);
  }

  pinMode(ECG_PIN, INPUT);

  if (!rtc.begin()) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("RTC Error!");
    while(1);
  }

  // Set RTC to your time
  rtc.adjust(DateTime(2025, 11, 29, 12, 25, 0));

  // First message
  lcd.setCursor(0,0);
  lcd.print("LoRa Transmitter");
  lcd.setCursor(0,1);
  lcd.print("will be heading");
  delay(5000);
  lcd.clear();
}

// Function to send SMS via SIM900A
void sendSMS(String number, String message){
  sim900.print("AT+CMGF=1\r"); // Text mode
  delay(100);
  sim900.print("AT+CMGS=\""+number+"\"\r");
  delay(100);
  sim900.print(message);
  delay(100);
  sim900.write(26); // Ctrl+Z to send
  delay(2000);
}

void loop() {
  // --- 1. Temperature ---
  double objectTemp = mlx.readObjectTempC();
  double ambientTemp = mlx.readAmbientTempC();

  lcd.setCursor(0,0);
  lcd.print("Ambient:");
  lcd.print(ambientTemp,2); lcd.print((char)223); lcd.print("C");

  lcd.setCursor(0,1);
  lcd.print("Object: ");
  lcd.print(objectTemp,2); lcd.print((char)223); lcd.print("C");
  delay(3000); lcd.clear();

  // --- 2. ECG ---
  int rawECG = analogRead(ECG_PIN);
  ecgValue = map(rawECG, 0, 4095, 0, 100);

  lcd.setCursor(0,0); lcd.print("ECG Raw: "); lcd.print(rawECG);
  lcd.setCursor(0,1); lcd.print("Mapped: "); lcd.print(ecgValue);
  delay(3000); lcd.clear();

  // --- 3. RTC ---
  DateTime now = rtc.now();
  lcd.setCursor(0,0);
  lcd.print("Date: "); lcd.print(now.day() < 10 ? "0":""); lcd.print(now.day());
  lcd.print("/"); lcd.print(now.month() <10?"0":""); lcd.print(now.month());
  lcd.print("/"); lcd.print(now.year());

  lcd.setCursor(0,1);
  lcd.print("Time: "); lcd.print(now.hour() <10?"0":""); lcd.print(now.hour());
  lcd.print(":"); lcd.print(now.minute()<10?"0":""); lcd.print(now.minute());
  lcd.print(":"); lcd.print(now.second()<10?"0":""); lcd.print(now.second());
  delay(3000); lcd.clear();

  // --- 4. Data Sent Screen ---
  lcd.setCursor(0,0);
  lcd.print("Data Sent to "); lcd.print(3); lcd.print(" Numbers  00");
  lcd.setCursor(0,1);
  lcd.print("1st "); lcd.print(phoneNumbers[0]); lcd.print("    01");
  
  // Send SMS to first 3 numbers (example)
  for(int i=0;i<3;i++){
    String msg = "Temp: "+String(objectTemp,2)+"C, ECG: "+String(ecgValue);
    sendSMS(phoneNumbers[i], msg);
  }

  delay(3000); lcd.clear();

  // --- 5. Numbers Screen ---
  lcd.setCursor(0,0); lcd.print(phoneNumbers[0]); lcd.print("              00");
  lcd.setCursor(0,1); lcd.print(phoneNumbers[1]); lcd.print("              01");
  delay(3000); lcd.clear();
}
