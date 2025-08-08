#include <Wire.h>
#include <MFRC522_I2C.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR_RFID 0x28
#define RST_PIN -1
MFRC522_I2C mfrc522(I2C_ADDR_RFID, RST_PIN, &Wire);

LiquidCrystal_I2C lcd(0x27, 20, 4);

byte validUID[4] = {0x61, 0x2E, 0x38, 0x02};

void setup() {
  Serial.begin(9600);
  Wire.setClock(100000);
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RFID Reader");
  lcd.setCursor(0, 1);
  lcd.print("Scan for RFID card");

  mfrc522.PCD_Init();
  delay(100);

  byte v = mfrc522.PCD_ReadRegister(0x37);
  Serial.print("Firmware Version: 0x");
  Serial.println(v, HEX);
}

void loop() {
  if (!mfrc522.PICC_IsNewCardPresent()) {
    delay(300);
    return;
  }

  if (!mfrc522.PICC_ReadCardSerial()) {
    delay(300);
    return;
  }

  String uidStr = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) uidStr += "0";
    uidStr += String(mfrc522.uid.uidByte[i], HEX);
    if (i < mfrc522.uid.size - 1) uidStr += " ";
  }
  uidStr.toUpperCase();

  Serial.print("Card UID: ");
  Serial.println(uidStr);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RFID Reader");
  lcd.setCursor(0, 1);
  lcd.print("Scan for RFID card");
  lcd.setCursor(0, 2);
  lcd.print("Card ID: ");
  lcd.print(uidStr);

  bool isValid = true;
  for (byte i = 0; i < 4; i++) {
    if (mfrc522.uid.uidByte[i] != validUID[i]) {
      isValid = false;
      break;
    }
  }

  lcd.setCursor(0, 3);
  if (isValid) {
    Serial.println("RFID confirmed: Welcome to RFID project");
    lcd.print("RFID Card Confirmed");
  } else {
    Serial.println("Wrong card!");
    lcd.print("Wrong Card!");
  }

  mfrc522.PICC_HaltA();
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("RFID Reader");
  lcd.setCursor(0, 1);
  lcd.print("Scan for RFID card");
}