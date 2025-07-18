#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "bmm150.h"
#include "bmm150_defs.h"

LiquidCrystal_I2C lcd(0x27, 20, 4);

bmm150_dev dev;
bmm150_settings settings;
bmm150_mag_data mag_data;

uint8_t bmm_addr = 0x13;
int prevX = 0, prevY = 0, prevZ = 0;
float prevHeading = -1;
String prevDirection = "";

int8_t i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
  Wire.beginTransmission(bmm_addr);
  Wire.write(reg_addr);
  Wire.endTransmission(false);
  Wire.requestFrom(bmm_addr, (uint8_t)len);
  for (uint32_t i = 0; i < len && Wire.available(); i++) {
    data[i] = Wire.read();
  }
  return 0;
}

int8_t i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
  Wire.beginTransmission(bmm_addr);
  Wire.write(reg_addr);
  for (uint32_t i = 0; i < len; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
  return 0;
}

void delay_us(uint32_t period, void *intf_ptr) {
  delayMicroseconds(period);
}

int prev_x = 0, prev_y = 0, prev_z = 0;
float prev_heading = -1;
String prev_direction = "";

void setup() {
  Serial.begin(9600);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  dev.intf_ptr = &bmm_addr;
  dev.intf = BMM150_I2C_INTF;
  dev.read = i2c_read;
  dev.write = i2c_write;
  dev.delay_us = delay_us;

  if (bmm150_init(&dev) != BMM150_OK) {
    lcd.setCursor(0, 1);
    lcd.print("BMM150 Init Fail!");
    Serial.println("BMM150 Init Failed");
    while (1);
  }

  settings.pwr_mode = BMM150_POWERMODE_NORMAL;
  bmm150_set_op_mode(&settings, &dev);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("   Compass");
}

void loop() {
  if (bmm150_read_mag_data(&mag_data, &dev) == BMM150_OK) {
    int x = mag_data.x;
    int y = mag_data.y;
    int z = mag_data.z;

    float heading = atan2((float)y, (float)x) * 180.0 / PI;
    if (heading < 0) heading += 360.0;

    String direction;
    if (heading >= 337.5 || heading < 22.5) direction = "North";
    else if (heading >= 22.5 && heading < 67.5) direction = "NE";
    else if (heading >= 67.5 && heading < 112.5) direction = "East";
    else if (heading >= 112.5 && heading < 157.5) direction = "SE";
    else if (heading >= 157.5 && heading < 202.5) direction = "South";
    else if (heading >= 202.5 && heading < 247.5) direction = "SW";
    else if (heading >= 247.5 && heading < 292.5) direction = "West";
    else direction = "NW";

    if (x != prev_x || y != prev_y || z != prev_z) {
      lcd.setCursor(0, 1);
      lcd.print("X:");
      lcd.print(x);
      
      lcd.print(" Y:");
      lcd.print(y);
   
      lcd.print(" Z:");
      lcd.print(z);
      

      prev_x = x;
      prev_y = y;
      prev_z = z;
    }

    if (abs(heading - prev_heading) >= 15) {
      lcd.setCursor(0, 2);
      lcd.print("Heading: ");
      lcd.print(heading, 1);
      lcd.print((char)223);
      lcd.print("     ");
      prev_heading = heading;
    }

    if (direction != prev_direction) {
      lcd.setCursor(0, 3);
      lcd.print("Dir: ");
      lcd.print(direction);
      lcd.print("        ");
      prev_direction = direction;
    }

    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Z: ");
    Serial.print(z);
    Serial.print(" Heading: ");
    Serial.print(heading);
    Serial.print(" Dir: ");
    Serial.println(direction);
  } else {
    Serial.println("Mag read error");
  }

  delay(300);
}
