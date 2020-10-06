#include "Adafruit_seesaw.h"

Adafruit_seesaw ss;

void setup() {
  Serial.begin(115200);

  Serial.println("seesaw Soil Sensor example!");
  
  if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }
}

//SAMO WYKONYWANIE POMIARÓW BEZ WYSYŁANIA NA TTN

void loop() {
  float tempC = ss.getTemp();
  uint16_t capread = ss.touchRead(0);
  uint16_t wynik= map(capread,338,1016,0,100);
  Serial.print("Temperature: "); Serial.print(tempC); Serial.println("*C");
  Serial.print("Capacitive: "); Serial.println(capread);
  Serial.print("Moisture %: "); Serial.println(wynik);
  delay(1000);
}
