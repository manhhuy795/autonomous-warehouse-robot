#include <Arduino.h>
#include <SoftwareSerial.h>

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// UNO D3 là TX gửi sang ESP32 RX
// UNO D2 là RX, không dùng nếu chỉ gửi 1 chiều
SoftwareSerial espSerial(2, 3); // RX, TX

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL_MS = 500;

float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0) {
    return -1.0;
  }

  return duration * 0.0343 / 2.0;
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(9600);
  espSerial.begin(9600);

  Serial.println("UNO ultrasonic sender started");
  Serial.println("Sending data to ESP32...");
}

void loop() {
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();

    float distance = readDistanceCm();

    if (distance < 0) {
      Serial.println("UNO -> DIST:ERROR");
      espSerial.println("DIST:ERROR");
    } 
    else {
      Serial.print("UNO -> DIST:");
      Serial.println(distance, 1);

      espSerial.print("DIST:");
      espSerial.println(distance, 1);
    }
  }
}