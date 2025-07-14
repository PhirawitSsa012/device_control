int countdownTime = 100;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.print("เริ่มนับถอยหลัง ");
  Serial.print(countdownTime);
  Serial.println(" วินาที...");
}

void loop() {
  for (int i = countdownTime; i >= 0; i--) {
    Serial.print("เหลือเวลา: ");
    Serial.print(i);
    Serial.println(" วินาที");
    delay(1000);
  }

  Serial.println("หมดเวลา!");


  while (true);
}