int countdownTime = 0; // ตัวแปรเก็บเวลานับถอยหลัง

void setup() {
  Serial.begin(9600);              // เริ่ม Serial
  while (!Serial);                 // รอจน Serial พร้อม (จำเป็นในบางบอร์ด เช่น Leonardo)

  Serial.println("กรุณาใส่เวลานับถอยหลัง (หน่วยเป็นวินาที):");

  while (Serial.available() == 0) {
    // รอจนกว่าผู้ใช้จะพิมพ์ตัวเลขเข้ามา
  }

  countdownTime = Serial.parseInt(); // อ่านค่าตัวเลขจาก Serial

  Serial.print("เริ่มนับถอยหลัง ");
  Serial.print(countdownTime);
  Serial.println(" วินาที...");
}

void loop() {
  for (int i = countdownTime; i >= 0; i--) {
    Serial.print("เหลือเวลา: ");
    Serial.print(i);
    Serial.println(" วินาที");
    delay(1000); // หน่วงเวลา 1 วินาที
  }

  Serial.println("หมดเวลา!");

  while (true); // หยุดโปรแกรมไว้ไม่ให้วนซ้ำ
}