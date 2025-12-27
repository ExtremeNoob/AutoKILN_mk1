#include <math.h>
#include <Wire.h>

//  Faire attention de fermer le moniteur serie sinon il bloque l'upload
void setup() {
}

float emc(float tc, float hrp){
  float w, k, k1, k2, tf, hr, emc;
//  float tc, hrp;                    // Temp en degres Celcius et humidite en %
  float Terme1, Terme2, Num, Denum;
  Serial.begin(115200);
  delay(500);
  Wire.begin();

  tf = 9/5*tc+32; hr = hrp/100;
  w = 330 + 0.452*tf + 0.00415*tf*tf;
  k = 0.791 + 0.000463*tf - 0.000000844*tf*tf;
  k1 = 6.34 + 0.000775*tf - 0.0000935*tf*tf;
  k2 = 1.09 + 0.0284*tf - 0.0000904*tf*tf;
  Terme1 = k*hr/(1 - k*hr);
  Num = k1*k*hr + 2*k1*k2*k*k*hr*hr;
  Denum = 1 + k1*k*hr + k1*k2*k*k*hr*hr;
  Terme2 = Num/Denum;
  emc = 1800/w * (Terme1 + Terme2);
  Serial.print("EMC = "); Serial.print(emc, 2); Serial.println("% \r"); 
  return emc;
}


void loop() {
  // put your main code here, to run repeatedly:
  emc(35, 90);
} 
