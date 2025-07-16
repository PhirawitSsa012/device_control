void setup() {
  Serial.begin(9600);

}

void loop() {
  Serial.println("พีรวิทย์ ศรีสะอาด");
  Serial.println("-------- Start -----------");
  for (int i = 1; i <= 4; i++) {
    for (int j = 1; j <= 4; j++) {
      if (i == j) {
        Serial.print(j);
        Serial.println(" is Green");
      } else {
        Serial.print(j);
        Serial.println(" is Red");
      }
    }
    Serial.println("-------------------");
    delay(1000);
  }
}