#include <ESP32Servo.h>

Servo myservo;   // ประกาศ Servo


void setup() {
  Serial.begin(9600);
  myservo.attach(32);  // กำหนดขาที่ใช้
  myservo.write(0);          // เริ่มที่มุม 0 องศา
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n'); 
    int angle = data.toInt(); 

    // จำกัดมุมให้อยู่ในช่วง 0–90 องศา
    angle = constrain(angle, 0, 90);

    // สั่ง Servo หมุน
    myservo.write(angle);

    // แสดงผลใน Serial Monitor
    Serial.print("มุมที่ได้รับ: ");
    Serial.println(angle);
  }
}