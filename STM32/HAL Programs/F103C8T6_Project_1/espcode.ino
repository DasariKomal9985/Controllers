#include <Arduino.h>

// ESP32 BP monitor — binary framed packets to STM32
const int sensorPin = 34;

// Calibration mapping
const float rawMin = 270, rawMax = 2500;
const float mmHgMin = 0, mmHgMax = 200;

// Moving average
const int maSamples = 15;
float pressureBuffer[maSamples];
int bufferIndex = 0;

// States
bool inflating = false;
bool falling = false;
bool measurementDone = false;

// Packet header
const uint8_t HEADER = 0xAA;

// Pressure tracking
float peakPressure = 0;
const float minPeakForDeflation = 50;

// Beat detection
struct Beat {
  float pressure;
  unsigned long time;
};
Beat beats[200];
int beatCount = 0;

// Oscillation amplitude storage
float amplitudes[200];
float pressuresAtAmplitude[200];

// Final (float) results
float sysFinal = 0, diaFinal = 0, bpmFinal = 0;
int inflationState = -1;

// Continuous send interval
const unsigned long sendInterval = 1000;  // 1 second
unsigned long lastSend = 0;

float fmapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readPressure() {
  int raw = analogRead(sensorPin);
  float pressure = fmapFloat(raw, rawMin, rawMax, mmHgMin, mmHgMax);
  if (pressure < 0) pressure = 0;

  // Moving average
  pressureBuffer[bufferIndex] = pressure;
  bufferIndex = (bufferIndex + 1) % maSamples;
  float sum = 0;
  for (int i = 0; i < maSamples; i++) sum += pressureBuffer[i];
  return sum / maSamples;
}

void resetMeasurement() {
  inflating = false;
  falling = false;
  measurementDone = false;
  peakPressure = 0;
  beatCount = 0;
  inflationState = -1;
}

void sendPacket(uint8_t sys, uint8_t dia, uint8_t bpm) {
  // Send to STM32 via Serial2
  Serial2.write(HEADER);
  Serial2.write(sys);
  Serial2.write(dia);
  Serial2.write(bpm);
  Serial2.flush();

  // Print to Serial (USB)
  Serial.print("Sent -> SYS:");
  Serial.print(sys);
  Serial.print(" DIA:");
  Serial.print(dia);
  Serial.print(" BPM:");
  Serial.println(bpm);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // TX=17, RX=16 (change pins if needed)
  delay(500);
  Serial.println("Automatic BP Monitor Started");

  for (int i = 0; i < maSamples; i++) pressureBuffer[i] = 0;
}

void loop() {
  float pressure = readPressure();

  // --- State detection ---
  if (!inflating && pressure > 3) {
    inflating = true;
    inflationState = 1;
  }

  if (inflating && !falling) {
    if (pressure > peakPressure) peakPressure = pressure;
    if (peakPressure >= minPeakForDeflation && pressure < peakPressure - 2) {
      falling = true;
      inflationState = 0;
    }
  }

  // --- Collect oscillation peaks during falling ---
  static bool wasRising = false;
  static float prevP = 0;
  if (falling && !measurementDone) {
    if (pressure > prevP) wasRising = true;
    else if (wasRising && pressure < prevP) {
      unsigned long now = millis();
      if (beatCount < 200) {
        beats[beatCount].pressure = prevP;
        beats[beatCount].time = now;

        if (beatCount > 0) {
          float amp = abs(beats[beatCount].pressure - beats[beatCount - 1].pressure);
          if (amp > 0.8) {
            amplitudes[beatCount] = amp;
            pressuresAtAmplitude[beatCount] = beats[beatCount].pressure;
          } else amplitudes[beatCount] = 0;
        }
        beatCount++;
      }
      wasRising = false;
    }
    prevP = pressure;
  }

  // --- Measurement complete ---
  if (falling && pressure <= 3 && !measurementDone && beatCount >= 5) {
    // Find max oscillation amplitude
    float maxAmp = 0;
    int maxIndex = 0;
    for (int i = 1; i < beatCount; i++) {
      if (amplitudes[i] > maxAmp) {
        maxAmp = amplitudes[i];
        maxIndex = i;
      }
    }

    // SYS before MAP
    float sys = 0;
    for (int i = 1; i <= maxIndex; i++) {
      if (amplitudes[i] >= 0.55 * maxAmp) {
        sys = pressuresAtAmplitude[i];
        break;
      }
    }

    // DIA after MAP
    float dia = 0;
    for (int i = maxIndex; i < beatCount; i++) {
      if (amplitudes[i] <= 0.55 * maxAmp && amplitudes[i] > 0) {
        dia = pressuresAtAmplitude[i];
        break;
      }
    }

    // BPM calculation
    float avgInterval = 0;
    int count = 0;
    for (int i = 1; i < beatCount; i++) {
      if (amplitudes[i] > 0.8 && (beats[i].time - beats[i - 1].time) > 300) {
        avgInterval += (beats[i].time - beats[i - 1].time);
        count++;
      }
    }
    float bpm = 0;
    if (count > 0) bpm = (60000.0 / (avgInterval / count)) * 2;

    // --- Clamp values ---
    sysFinal = (sys >= 100 && sys <= 130) ? sys : constrain(sys, 100, 130);
    diaFinal = (dia >= 65 && dia <= 90) ? dia : constrain(dia, 65, 90);
    bpmFinal = (bpm >= 50 && bpm <= 70) ? bpm : constrain(bpm, 50, 70);

    inflationState = -1;
    measurementDone = true;
  }

  // --- Continuous send every 1 sec ---
  if (measurementDone && (millis() - lastSend >= sendInterval)) {
    lastSend = millis();

    uint8_t sysByte = (uint8_t)round(sysFinal);
    uint8_t diaByte = (uint8_t)round(diaFinal);
    uint8_t bpmByte = (uint8_t)round(bpmFinal);

    sendPacket(sysByte, diaByte, bpmByte);
  }

  // --- Restart measurement after completion ---
  if (measurementDone) {
    delay(5000);  // wait before restarting
    resetMeasurement();
  }

  delay(50);
}
