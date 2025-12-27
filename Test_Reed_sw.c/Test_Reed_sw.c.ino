/**********************************************************************
* Filename    : ButtonAndLed
* Description : Use a button to control LED light
* Auther      : www.freenove.com
* Modification: 2021/10/13
**********************************************************************/
#include <Wire.h>
#include <pico_GPIO_Number.h>

//----- Includes et constantes pour la balance ------------------------
#include "HX711.h"
// #include "pico_GPIO_Number.h"
const int DOUT_PIN = GPIO11;   // GP4
const int SCK_PIN  = GPIO10;   // GP5
#define CALIBRATION_FACTOR -7050.0




#define DOOR GPIO0
#define HEATER GPIO16
#define DEHUMIDIFYER GPIO17
#define DIGITALSCALE 999

#define TRUE HIGH
#define FALSE LOW
#define ON 1
#define OFF 0


class TCapteur{
  public:
    TCapteur(int aGPIO);
    bool LectureBinaire();
    float LectureLineaire();
  protected:
    int gpio;
    float value;
  private:
  };

TCapteur::TCapteur(int aGPIO){
  gpio = aGPIO;
  pinMode(aGPIO, INPUT);
  }

bool TCapteur::LectureBinaire(){
  value = digitalRead(gpio);
  return (bool)value;
  }


  
class TPorte : public TCapteur{
  public:
    TPorte(int aGPIO);
    bool IsOpen();
    bool IsClosed(); 
  protected:  
    bool status;
    int gpio;
};

TPorte::TPorte(int aGPIO) : TCapteur(aGPIO){}

//bool TPorte::ReadSensor(){ return (bool)digitalRead(DOOR); }
bool TPorte::IsOpen(){ return LectureBinaire(); }
bool TPorte::IsClosed(){ return !LectureBinaire(); }


class TBalance : public TCapteur{
  public:
    TBalance(int aGPIO);
    float LirePoids();
    void FaireTare();
    HX711 scale;
  protected:
    float calibration_factor; 
  private:
  };

TBalance::TBalance(int aGPIO) : TCapteur(aGPIO) { 
  calibration_factor = CALIBRATION_FACTOR; // placeholder: you will change this
  scale.begin(DOUT_PIN, SCK_PIN);
  scale.set_scale(calibration_factor);   // raw
  scale.tare();        // zero the scale
  Serial.println("HX711 ready. Remove all weight. Tare done.\n");
  delay(2000);
}

float TBalance::LirePoids(){
  Serial.println("Inside LirePoids.\n");
  if (!scale.is_ready()) {
    Serial.println("HX711 NOT ready (DOUT HIGH). Check wiring/pins/power.");
    delay(200);
    return NAN; // ou 0
  }

  float v = scale.get_units(10);
  Serial.print("Poids: ");
  Serial.println(v, 2);
  return v;
  }

void TBalance::FaireTare(){
  scale.tare();
  }

//---------------------------------------------------------------------

class TActionneur{
  public:
    TActionneur(int aGPIO);
  protected:
    void SortieBinaire(bool aState);
    void SortieParallele(int aValue);
    void SortieSerie(int aValue);
    int gpio;
    int etat;
  private:
  };

TActionneur::TActionneur(int aGPIO){
    gpio = aGPIO;
    pinMode(aGPIO, OUTPUT);
  }

void TActionneur::SortieBinaire(bool aState){
    digitalWrite(gpio,aState);
  }  

void TActionneur::SortieParallele(int aValue){
//    digitalWrite(gpio,aValue);
  }  
 
void TActionneur::SortieSerie(int aValue){
//   digitalWrite(gpio,aValue);
  }  


class TChaufferette : public TActionneur{
  public:
    TChaufferette(int aGPIO);
    void TurnOn();
    void TurnOff();
  private:
  };
  
TChaufferette::TChaufferette(int aGPIO):TActionneur(aGPIO){}
  
void TChaufferette::TurnOn(){ 
  digitalWrite(gpio,ON); 
  //digitalWrite(LED_BUILTIN,HIGH);
}
void TChaufferette::TurnOff(){ 
  digitalWrite(gpio,OFF); 
//  digitalWrite(LED_BUILTIN,LOW);
  }


class TDeshumidificateur : public TActionneur{
  public:
    TDeshumidificateur(int aGPIO);
    void TurnOn();
    void TurnOff();
  private:
  };

TDeshumidificateur::TDeshumidificateur(int aGPIO):TActionneur(aGPIO){}

void TDeshumidificateur::TurnOn(){ 
  digitalWrite(gpio,ON); 
//  digitalWrite(LED_BUILTIN,HIGH);
  }
void TDeshumidificateur::TurnOff(){ 
  digitalWrite(gpio,OFF); 
//  digitalWrite(LED_BUILTIN,LOW);
  }

 void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  pinMode(15, OUTPUT);          // initialize digital pin BUILTIN_LED as an output.
}
  TPorte Porte(DOOR);
  TChaufferette Chaufferette(HEATER);
  TDeshumidificateur Deshumidificateur(DEHUMIDIFYER);
  TBalance Balance(21);
   
void loop() {

  
  float test = Balance.LirePoids();
  
  //Serial.println("  Poids= "); 
  //Serial.println(Balance.LirePoids(),2);
  //Serial.println(4.456,2);
 
  if (Porte.IsOpen()){
 //   digitalWrite(LED_BUILTIN,HIGH);
      Chaufferette.TurnOn();
      Deshumidificateur.TurnOn();
  }else{
    //digitalWrite(LED_BUILTIN,LOW);
    Chaufferette.TurnOff();
    Deshumidificateur.TurnOff();

  }
  delay(2000);
}
