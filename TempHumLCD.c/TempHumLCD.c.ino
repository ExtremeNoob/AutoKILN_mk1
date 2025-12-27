


/**********************************************************************
  Filename    : Drive LiquidCrystal I2C to display characters
  Description : I2C is used to control the display characters of LCD1602.
  Auther      : www.freenove.com
  Modification: 2021/10/13
**********************************************************************/
#include <LiquidCrystal_I2C.h>
#include <dht.h>
#include "pico_GPIO_Number.h"
#include <HX711.h>
#include <math.h>

#define DHT11_PIN GPIO15
#define POLLING_DELAY 2000

LiquidCrystal_I2C lcd(0x27,16,2); 

dht DHT;
const int DOUT_PIN = GPIO12;   // GP4
const int SCK_PIN  = GPIO11;   // GP5

HX711 scale;

float calibration_factor = -22195; // placeholder: you will change this
void setup_scale() {
  Serial.begin(115200);
  delay(1000);

  scale.begin(DOUT_PIN, SCK_PIN);
  scale.set_scale(calibration_factor);   // raw
  scale.tare();        // zero the scale
  Serial.println("HX711 ready. Remove all weight. Tare done.");
}

void setup() {
  if (!i2CAddrTest(0x27)) {
    lcd = LiquidCrystal_I2C(0x3F, 16, 2);
  }
  lcd.init();                     // LCD driver initialization
  lcd.backlight();                // Open the backlight
  lcd.setCursor(0,0);             // Move the cursor to row 0, column 0
  setup_scale();
}

void loop() {
  int chk = DHT.read11(DHT11_PIN);
  float kg;
  
  if(chk == DHTLIB_OK){
    lcd.setCursor(0,0);             // Move the cursor to row 1, column 0
    lcd.print("H.: " + String(DHT.humidity) + "%");
    lcd.setCursor(0,1);             // Move the cursor to row 1, column 0
    lcd.print("T.: " + String(DHT.temperature) + "C");
  }else {
    lcd.setCursor(0,0);             // Move the cursor to row 1, column 0
    lcd.print("DHT11 Reading data error!");
  }
  if (scale.is_ready()) {
//    float kg = scale.get_units(10);
    kg = scale.get_units(10);
    kg = fabsf(kg);         // Removes the negative value
    Serial.println(kg, 3);  // 2e parametre est le nombre de chiffres apres la virgule
  }else {
    Serial.println("HX711 not ready (check wiring/power)");
  }
  delay(POLLING_DELAY);
}

bool i2CAddrTest(uint8_t addr) {
  Wire.begin();
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() == 0) {
    return true;
  }
  return false;
}
