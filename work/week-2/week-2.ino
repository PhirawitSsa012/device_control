#define LED_BUILTIN 2
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //print to monitor becuae set begin
  
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  Serial.print("สวัสดี พีรวิทย์ ศรีสะอาด\n");

  digitalWrite(LED_BUILTIN, 1);
  Serial.print("light ... "); 
  delay(1000);

  digitalWrite(LED_BUILTIN, 0);
  Serial.print("no light ... "); 
  delay(1000);
}