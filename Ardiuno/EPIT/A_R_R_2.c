
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5

// GPS Configuration
static const int RXPin = 6, TXPin = 7;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// Compass Configuration
QMC5883LCompass compass;
float heading, targetHeading;
float targetLat = 0.0;
float targetLong = 0.0;
float espLat, espLon;
bool obstacleDetected = false;

// NRF24 Configuration
RF24 radio(8, 9);
const byte address[6] = "00001";
bool control_state = 1; // 1 = Manual (NRF), 2 = Autonomous (GPS)

int pump_pin = 10;
int pump_state = 0;
struct MotorData {
    int motor1;
    int motor2;
    int motor3;
    int motor4;
    int command;
    int servo;
};

MotorData receivedData;

int obstacle;

void stop() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void forward() {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void left() {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void right() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void moveBackward() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = radians(lon2 - lon1);
    float y = sin(dLon) * cos(radians(lat2));
    float x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
    return fmod(degrees(atan2(y, x)) + 360, 360);
}

void setup() {
    Serial.begin(9600);
    gpsSerial.begin(GPSBaud);
    Wire.begin();
    compass.init();
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    stop();

    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
    Serial.println("Receiver Initialized");
}

void loop() {
    control_state = receivedData.servo;
    pump_state = receivedData.command;

    if (pump_state) {
        digitalWrite(pump_pin, LOW);
        delay(100);
    } else {
        digitalWrite(pump_pin, HIGH);
        delay(100);
    }

    if (Serial.available()) {
        String espData = Serial.readStringUntil('\n');
        int detected;
        if (parseESPData(espData, espLat, espLon, detected)) {
            targetLat = espLat;
            targetLong = espLon;
            obstacleDetected = detected;
            Serial.print("Updated Target Latitude: "); Serial.println(targetLat, 6);
            Serial.print("Updated Target Longitude: "); Serial.println(targetLong, 6);
            Serial.print("Obstacle Detected: "); Serial.println(obstacleDetected ? "Yes" : "No");
            obstacle = obstacleDetected ? 1 : 0;
        } else {
            Serial.println("Invalid ESP data received.");
        }
    }

    if (radio.available()) {
        radio.read(&receivedData, sizeof(receivedData));
        Serial.print("Motor1: "); Serial.print(receivedData.motor1);
        Serial.print(" | Motor2: "); Serial.print(receivedData.motor2);
        Serial.print(" | Motor3: "); Serial.print(receivedData.motor3);
        Serial.print(" | Motor4: "); Serial.print(receivedData.motor4);
        Serial.print(" | Servo: "); Serial.print(receivedData.servo);
        Serial.print(" | Command: "); Serial.println(receivedData.command);

        if (control_state == 0) {
            if (!obstacle) {
                controlMotors();
                Serial.println("remote control");
            } else {
                stop();
            }
        }
    }

    if (control_state == 1) {
        Serial.println("autonomous");
        while (gpsSerial.available() > 0) {
            gps.encode(gpsSerial.read());
        }
        if (gps.location.isValid()) {
            float currentLat = gps.location.lat();
            float currentLon = gps.location.lng();
            targetHeading = calculateBearing(currentLat, currentLon, targetLat, targetLong);
            compass.read();
            int heading = compass.getAzimuth();
            if (heading < 0) heading += 360;

            float angleDiff = targetHeading - heading;
            if (angleDiff < -180) angleDiff += 360;
            if (angleDiff > 180) angleDiff -= 360;

            if (!obstacle) {
                if (abs(angleDiff) < 20) {
                    forward();
                } else if (angleDiff > 20) {
                    right();
                } else {
                    left();
                }
            } else {
                stop();
            }

            if (gps.distanceBetween(currentLat, currentLon, targetLat, targetLong) < 2.0) {
                stop();
                Serial.println("Destination reached!");
                while (1);
            }
        }
    }
}

void controlMotors() {
    if (receivedData.motor1 == 1 && receivedData.motor2 == 1 && receivedData.motor3 == 1 && receivedData.motor4 == 1) {
        forward();
    } else if (receivedData.motor1 == -1 && receivedData.motor2 == -1 && receivedData.motor3 == -1 && receivedData.motor4 == -1) {
        moveBackward();
    } else if (receivedData.motor1 == -1 && receivedData.motor3 == -1 && receivedData.motor2 == 1 && receivedData.motor4 == 1) {
        right();
    } else if (receivedData.motor1 == 1 && receivedData.motor3 == 1 && receivedData.motor2 == -1 && receivedData.motor4 == -1) {
        left();
    } else {
        stop();
    }
}

bool parseESPData(String data, float &lat, float &lon, int &obstacle) {
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
    if (firstComma == -1 || secondComma == -1) return false;
    lat = data.substring(0, firstComma).toFloat();
    lon = data.substring(firstComma + 1, secondComma).toFloat();
    obstacle = data.substring(secondComma + 1).toInt();
    return true;
}
