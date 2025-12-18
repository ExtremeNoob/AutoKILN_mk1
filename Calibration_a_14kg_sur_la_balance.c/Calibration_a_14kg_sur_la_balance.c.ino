#include "HX711.h"

const int DOUT_PIN = 12;   // GP4
const int SCK_PIN  = 11;   // GP5
const float KNOWN_KG = 14.0;

HX711 scale;

void setup() {
  Serial.begin(115200);
  delay(1000);

  scale.begin(DOUT_PIN, SCK_PIN);

  Serial.println("\nHX711 calibration");
  Serial.println("1) Remove all weight. Taring in 3 seconds...");
  delay(3000);

  scale.tare(); // sets current load as zero
  Serial.println("Tare done.");

  Serial.println("2) Place the 14kg weight NOW. Reading in 3 seconds...");
  delay(3000);

  long reading = scale.get_value(20); // raw value minus tare (avg of 20)
  Serial.print("Net raw reading: ");
  Serial.println(reading);

  float cal = (float)reading / KNOWN_KG; // counts per kg
  Serial.print("Suggested set_scale factor: ");
  Serial.println(cal, 3);

  Serial.println("\nUse it like:");
  Serial.println("scale.set_scale(<that number>);");
  Serial.println("scale.tare();");
}

void loop() {}
