//----------------------------- Ne pas toucher ce code.  Bon pour verification --------

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_SHT31.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);   // change if your LCD address differs
Adafruit_SHT31 sht31;

void setup() {
  Wire.begin();

  lcd.init();
  lcd.backlight();

  if (!sht31.begin(0x44) && !sht31.begin(0x45)) {
    lcd.clear();
    lcd.print("SHT31 not found");
    while (1) delay(10);              // stops and loop endlessly here
  }
}

void loop() {
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();

  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(t, 1);
  lcd.print((char)223); // degree symbol on many 1602 ROMs
  lcd.print("C    ");

  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(h, 1);
  lcd.print("%      ");

  delay(1000);
}
