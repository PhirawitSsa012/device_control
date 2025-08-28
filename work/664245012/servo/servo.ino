#include <ESP32Servo.h>
Servo myservo;

int ledPin = 2;
unsigned long prevMillis = 0;
int ledState = LOW;
int blinkInterval = 0;  // ควบคุมความเร็วการกระพริบ
int angle = 0;
bool isStopped = true;

void setup() {
  Serial.begin(9600);
  myservo.attach(32);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); 
    command.trim();

    if (command == "stop" || command == "0") {
      Serial.println("Servo Motor => stop");
      myservo.write(0);
      digitalWrite(ledPin, LOW);   // ดับไฟ
      isStopped = true;
    } 
    else {
      angle = command.toInt();
      if (angle >= 0 && angle <= 180) {
        Serial.print("Servo Motor => ");
        Serial.println(angle);
        myservo.write(angle);
        digitalWrite(ledPin, HIGH);
        delay(1000);
        digitalWrite(ledPin, LOW);
        isStopped = false;

        // กำหนดรูปแบบไฟตามเงื่อนไข
        if (angle > 110 && angle <= 140) {
          digitalWrite(ledPin, HIGH);   // ไฟติดค้าง
          blinkInterval = 0;
        } 
        else if (angle > 140) {
          blinkInterval = 3000;         // กระพริบช้า (2 วิ)
        } 
        else {
          blinkInterval = 0;          // กระพริบเร็ว
        }
      }
    }
  }

  // ควบคุมไฟกระพริบแบบ non-blocking
  if (!isStopped && blinkInterval > 0) {
    unsigned long currentMillis = millis();
    if (currentMillis - prevMillis >= blinkInterval) {
      prevMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
    }
  }
}
