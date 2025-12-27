#include <Wire.h>
#include "Adafruit_SHT31.h"

Adafruit_SHT31 sht31 = Adafruit_SHT31();

void setup() {
  Serial.begin(115200);
  delay(500);

  // If your LCD already works on I2C, Wire is probably fine as-is.
  Wire.begin();

  if (!sht31.begin(0x44)) {          // try 0x44 first
    Serial.println("SHT31 not found at 0x44. Trying 0x45...");
    if (!sht31.begin(0x45)) {
      Serial.println("SHT31 not found (check wiring / address).");
      while (1) delay(10);
    }
  }
  Serial.println("SHT31 OK");
}

void loop() {
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  Serial.print("T="); Serial.print(t, 2);
  Serial.print(" C  H="); Serial.print(h, 2);
  Serial.println(" %");

  delay(1000);
}
