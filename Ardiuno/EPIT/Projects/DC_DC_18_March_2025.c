

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize the LCD (I2C Address: 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define VOLTAGE_SENSOR_PIN1 A0  // First voltage sensor (A0) - Should show 12V
#define VOLTAGE_SENSOR_PIN2 A1  // Second voltage sensor (A1) - Should show 14V

// Resistor values
const float R1 = 100000.0; // 100kΩ
const float R2 = 10000.0;  // 10kΩ

// Actual measured reference voltage (use a multimeter to check)
const float REF_VOLTAGE = 1.085; // Example: Measured as 1.085V instead of 1.1V
const int ADC_RESOLUTION = 1023; // 10-bit ADC

// Corrected scaling factors
const float SCALING_FACTOR_A0 = 1.00; // Keep A0 normal to show 12V
const float SCALING_FACTOR_A1 = 1.17; // Increase A1 to show 14V correctly

void setup() {
    lcd.begin();
    lcd.backlight();
    Serial.begin(9600);
    
    // Use internal 1.1V reference instead of 5V
    analogReference(INTERNAL);

    lcd.setCursor(0, 0);
    lcd.print("Voltage Sensor");
    delay(2000);
}

// Function to get accurate voltage reading with averaging & correction
float readVoltage(int pin, float scalingFactor) {
    long sum = 0;
    int samples = 10; // Take 10 samples for accuracy

    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
        delay(5);
    }
    
    float avgSensorValue = sum / samples;
    float vout = (avgSensorValue * REF_VOLTAGE) / ADC_RESOLUTION;
    float vin = vout * ((R1 + R2) / R2) * scalingFactor; // Apply separate scaling factor
    
    if (vin < 0.1) vin = 0.0; // Remove floating values
    
    return vin;
}

void loop() {
    // Read and calculate voltages separately with corrected scaling
    float vin1 = readVoltage(VOLTAGE_SENSOR_PIN1, SCALING_FACTOR_A0); // A0 should show 12V
    float vin2 = readVoltage(VOLTAGE_SENSOR_PIN2, SCALING_FACTOR_A1); // A1 should show 14V

    // Calculate Gain (V2 / V1)
    float gain = (vin1 > 0.1) ? (vin2 / vin1) : 0; // Avoid division by zero

    // Display voltages on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("V1: ");
    lcd.print(vin1, 2);
    lcd.print("V");

    lcd.setCursor(0, 1);
    lcd.print("V2: ");
    lcd.print(vin2, 2);
    lcd.print("V");

    delay(2000); // Show voltages for 2 seconds

    // Display Gain on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gain: ");
    lcd.print(gain, 2);

    // Print to Serial Monitor
    Serial.print("V1: ");
    Serial.print(vin1, 2);
    Serial.print(" V | ");

    Serial.print("V2: ");
    Serial.print(vin2, 2);
    Serial.print(" V | ");

    Serial.print("Gain: ");
    Serial.println(gain, 2);

    delay(2000); // Show Gain for 2 seconds before refreshing
}
\