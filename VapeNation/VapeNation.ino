//Basic code für Vape
//OHNE Display, OHNE Ohmmeter für Coil
//Der Code ist nicht getestet und von Elektrotechnik(Volt,Ampere,Ohm,Watt) hab ich nicht wirklich ein plan
//deswegen sollst du dir dat mal anschauen und dann können wir das mal durchgehen

#include <EEPROM.h>

//Pinbelegung
#define FIREBUTTON  2   //Feuerknopf | Digital Input
#define VBAT_IN     A0  //Battery Voltage | Analog Input
#define VMB_IN      A1  //Voltage zwischen den beiden Zellen, zur berechnung von jeweiligem Akkustand
#define GATE_PIN    8   //Mosfet Gate | PWM Output
#define RE_CLK      3   //Drehgeber Clock
#define RE_DT       4   //Drehgeber Data
#define RE_BUTTON   5   //Drehgeber Button

//Vape Begrenzungen
#define LockTime  400 //=Zeit in ms die max. zwischen den klicks liegt (für die 5klick sperre)
#define MinWatt   10
#define MaxWatt   150
#define MinOhm    0.1
#define MaxOhm    2.0
#define MaxAmp    35

//Globale Variablen
float CoilOhm = 0.2f;

//Funktionsprototypen
byte rotaryEncoder();
float getVBat (float *A = NULL, float *B = NULL);
void fire(int Watt);


void setup()
{
  pinMode(FIREBUTTON, INPUT);
  pinMode(VBAT_IN, INPUT);
  pinMode(VMB_IN, INPUT);
  pinMode(GATE_PIN, OUTPUT);
  pinMode(RE_CLK, INPUT);
  pinMode(RE_DT, INPUT);
  pinMode(RE_BUTTON, INPUT);
  
  Serial.begin(9600);
}


void loop()
{
  //5 Klick Tastensperre
  static bool bLock = true;
  static int tLastFire = millis();
  static byte counter = 0;
  if(counter == 5)
  {
    bLock = !bLock;
    counter = 0;
    Serial.println("Tastensperre umgeschaltet");
  }
  
  //Watteinstellung
  static int Watt = 0;
  if(Watt == 0)
    EEPROM.get(0, Watt);
  switch(rotaryEncoder())
  {
      case(1):
      Watt++;
      EEPROM.put(0, Watt);
      break;
      case(2):
      Watt--;
      EEPROM.put(0, Watt);
      break;
  }
  if(Watt<MinWatt)
    Watt=MinWatt;
  if(Watt>MaxWatt)
    Watt=MaxWatt;

  //Feuerknopf
  if(digitalRead(FIREBUTTON) == HIGH)
  {
    //Für Tastensperre
    int t = millis() - tLastFire;
    if( t <= LockTime && t > 10 ) // >10 = Tasterentprellung
      counter++;
    if( t > LockTime)
      counter=0;
    tLastFire = millis();
    
    //Vaperino
    while(digitalRead(FIREBUTTON) == HIGH && !bLock) //AUFPASSEN: Öffner oder Schließer  
    {
      fire(Watt);

      if(millis()-tLastFire > 10000)  //Wenn Feuertaste länger als 10s gedrückt
      {
        bLock = true; //Sperre aktivieren, sonst feuert vape ja direkt wieder
        continue;     //Aus while-Schleife springen
      } 
    }
    analogWrite(GATE_PIN, 0); //Hätte ich fast vergessen
  }
}


void fire(int Watt)
{  
  float vBat, vBatA, vBatB;
  vBat = getVBat(&vBatA,&vBatB);
  
  //Akkustand prüfen
  if(vBatA < 3.4f || vBatB < 3.4f)
  {
    //Display: Battery Low
    Serial.print("Battery Low | A: "); Serial.print(vBatA); Serial.print("V, B: "); Serial.print(vBatB); Serial.print("V\n");
    return;
  }
  
  //CoilOhm prüfen
  if(CoilOhm < MinOhm)
  {
    //Display: Ohm too low
    Serial.print("Ohm 2low: "); Serial.print(CoilOhm); Serial.print("Ω\n");
    return;
  }
  if(CoilOhm > MaxOhm)
  {
    //Display: Ohm too high
    Serial.print("Ohm 2high: "); Serial.print(CoilOhm); Serial.print("Ω\n");
    return;
  }
  
  //Berechnung der Ausgangsspannung (Drain vom Mosfet)
  float vOut = sqrt( Watt * CoilOhm ); //Spannung die durch den Coil geht | Ohmsches Gesetz U=√P*R

  //Stromstärke begrenzen wenn zu hoch
  float amp = vOut / CoilOhm; //Frage: Ist das dann die Stromstärke die aus den Akkus gezogen wird? Ich denk schon
  if(amp > MaxAmp)
  {
    //Display: Ampere too high
    vOut = CoilOhm * MaxAmp;
  } //Nach meinen berechnungen springt das eh nur ein wenn man über zweihundertundirgendwas watt dampft oder mach ich hier was falsch
  
  //Wenn ich pwm+mosfet richtig verstanden habe gate=0->Aus & gate=255->100% von Batteriespannung
  int gate;
  if (vBat > vOut) //Kann reguliert werden
    gate =  ( 255 / vBat ) * vOut;
  else
  {
    //Display: Weak Battery
    //Watt = (vBat*vBat)/CoilOhm
    gate = 255; //Geb volle pulle
  }
  analogWrite(GATE_PIN, gate);
}

float getVBat (float *A, float *B)
/* FAQ:
Q: Was sind die Sternchen? A: Pointer
Q: Und warum machen wir das nicht einfach über globale Variablen? A: Weil das schlechter programmierstil ist
Danke für ihre Aufmerksamkeit */
{ //https://www.electroschematics.com/9351/arduino-digital-voltmeter/
  
  //Für höhere genauigkeit müssen wir Ohm der Widerstände 1x ausmessen und anpassen
  float R1 = 100000.f;
  float R2 = 10000.f;

  //Gesamt Voltage berechnen
  int value = analogRead(VBAT_IN);
  float vout = (value * 5.0) / 1024.0; //Für höhere genauigkeit müssen wir die Arduino 5v Spannung messen und anpassen (bei 5.0)
  float vBat = vout / ( R2 / (R1+R2) );

  if(A != NULL && B!= NULL) //Beide Zellen berechnen
  {
    value = analogRead(VMB_IN);
    vout = (value * 5.0) / 1024.0;
    float vA = vout / ( R2 / (R1+R2) );
    float vB = vBat - vA;
    *A = vA;
    *B = vB;
  }

  return vBat;
}

byte rotaryEncoder()
{
  //Rückgabewerte:
  //0: keine Rotation
  //1: im Uhrzeigersinn
  //2: gegen Uhrzeigersinn
  
  byte ret = 0;
  static int lastState = digitalRead(RE_CLK);
  
  int state = digitalRead(RE_CLK);
  if(state != lastState)
  {
    if(digitalRead(RE_DT) != state) //clockwise
      ret = 1;
    else  //counter-clockwise
      ret = 2;
  }
  lastState = state;
  return ret;
}

