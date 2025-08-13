#include "DHT.h"

#define DHPIN 13
#define DHTTYPE DHT11

DHT dht(DHPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (!isnan(temp) && !isnan(hum)) {
    Serial.print("Temp:");
    Serial.print(temp);
    Serial.print(";Hum:");
    Serial.println(hum);  
  }

  delay(2000);  // รอ 2 วินาที (2000 มิลลิวินาที)
}