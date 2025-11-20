const int pHSensorPin = A11;
float offset = 0.0;  // adjust after pH 7 calibration
float slope  = 3.5;  // adjust after pH 4 calibration

void setup() {
  Serial.begin(9600);
}

void loop() {
  int adcValue = analogRead(pHSensorPin);
  float voltage = adcValue * 5.0 / 1023.0; 
  float pHValue = slope * voltage + offset;

  Serial.print("ADC: "); Serial.print(adcValue);
  Serial.print("  V: "); Serial.print(voltage, 4);
  Serial.print("  pH: "); Serial.println(pHValue, 3);

  delay(500);
}
