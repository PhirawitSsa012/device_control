#include <ESP32Servo.h>
Servo myservo;
char user_input;

void setup() {
  Serial.begin(9600);
  myservo.attach(5);
}

void loop() {
  if (Serial.available() > 0) {
    user_input = Serial.read();

    if (user_input == '1') {
      Serial.println("Servo Motor => 120");
      myservo.write(120);
    } 
    else if (user_input == '2') {
      Serial.println("Servo Motor => 150");
      myservo.write(150);
    } 
    else if (user_input == '3') {
      Serial.println("Servo Motor => 180");
      myservo.write(180);
    } 
    else if (user_input == '4') {
      Serial.println("Servo Motor => 45");
      myservo.write(45);
    }
    delay(1000);
  }
}
