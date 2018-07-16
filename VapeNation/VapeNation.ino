//Code für Vape
//OHNE Display, OHNE Ohmmeter für Coil
//Der Code ist nicht getestet und von Elektrotechnik(Volt,Ampere,Ohm,Watt) hab ich nicht wirklich ein plan
//deswegen sollst du dir dat mal anschauen und dann können wir das mal durchgehen

#include <EEPROM.h>

//Pinbelegung
#define FIREBUTTON  2   //Feuerknopf | Digital Input
#define VBAT_IN     A0  //Battery Voltage | Analog Input
#define VMB_IN      A1  //Voltage zwischen den beiden Zellen, zur berechnung von jeweiligem Akkustand
#define GATE_PIN    9   //Mosfet Gate | PWM Output
#define RE_CLK      3   //Drehgeber Clock
#define RE_DT       4   //Drehgeber Data
#define RE_BUTTON   5   //Drehgeber Button

//Vape Begrenzungen
#define MCTIME     400 //Multi-Click-Time (ms)
#define WATT_MIN   10
#define WATT_MAX   150
#define OHM_MIN    0.1
#define OHM_MAX    2.0
#define AMP_MAX    35

//EEPROM Adressen (0-1024)
#define ADR_WATT  0 //int
#define ADR_PUFFS 4 //int
#define ADR_OHM   8 //float

//Funktionsprototypen
byte rotaryEncoder();
float getVBat (float *A = NULL, float *B = NULL);
void fire(int Watt, float CoilOhm);
bool query(String question);
float getCoilOhm()
{
  //tbd; Soll ohm am coil messen
  return 0.f;
}

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
  static unsigned long tLastFire = millis(); //Speichert die Zeit an welcher der Feuerknopf zuletzt gedrückt wurde
  
  //5 Klick Tastensperre
  static bool bLock = true;
  static byte counter = 0;
  if(counter == 5)
  {
    bLock = !bLock;
    counter = 0;
    Serial.println("Tastensperre umgeschaltet");
  }
  
  //Watteinstellung
  static int watt = 0;
  if(watt == 0)
    EEPROM.get(ADR_WATT, watt);
  switch(rotaryEncoder())
  {
      case(1):
      watt++;
      EEPROM.put(ADR_WATT, watt);
      break;
      case(2):
      watt--;
      EEPROM.put(ADR_WATT, watt);
      break;
  }
  if(watt<WATT_MIN)
    watt=WATT_MIN;
  if(watt>WATT_MAX)
    watt=WATT_MAX;
  
  
  //Coil Widerstand
  float coilohm = -1.f;/*
  if(coilohm == -1.f)
    EEPROM.get(ADR_OHM, coilohm);
  if( coilohm != 0.00f && ( (coilohm - getCoilOhm()) >= 0.1f || (coilohm - getCoilOhm()) <= -0.1f) ))
  {
    if(query("Neuer Coil?"))
    {
      coilohm = getCoilOhm();
      EEPROM.put(ADR_OHM, coilohm);
    }  
  }
  else if(getCoilOhm
  */ 

  //Züge
  static int puffs = -1;
  if(puffs == -1)
    EEPROM.get(ADR_PUFFS, puffs);
  
  //Feuerknopf
  if(digitalRead(FIREBUTTON) == HIGH)
  {
    //Für Tastensperre
    int t = millis() - tLastFire;
    if( t <= MCTIME && t > 10 ) // >10 = Tasterentprellung
      counter++;
    if( t > MCTIME)
      counter=0;
    tLastFire = millis();
    
    //Vaperino
    while(digitalRead(FIREBUTTON) == HIGH && !bLock) //AUFPASSEN: Öffner oder Schließer  
    {
      fire(watt, coilohm);

      if(millis()-tLastFire > 10000)  //Wenn Feuertaste länger als 10s gedrückt
        bLock = true; //Sperre aktivieren
    }
    analogWrite(GATE_PIN, 0); //Hätte ich fast vergessen
    
    //Für Züge
    if(millis()-tLastFire >= 1000)
    {
      puffs++;
      EEPROM.put(ADR_PUFFS, puffs);
    }
  }
}


void fire(int Watt, float CoilOhm)
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
  if(CoilOhm < OHM_MIN)
  {
    //Display: Ohm too low
    Serial.print("Ohm 2low: "); Serial.print(CoilOhm); Serial.print("Ω\n");
    return;
  }
  if(CoilOhm > OHM_MAX)
  {
    //Display: Ohm too high
    Serial.print("Ohm 2high: "); Serial.print(CoilOhm); Serial.print("Ω\n");
    return;
  }
  
  //Berechnung der Ausgangsspannung (Drain vom Mosfet)
  float vOut = sqrt( Watt * CoilOhm ); //Spannung die durch den Coil geht | Ohmsches Gesetz U=√P*R

  //Stromstärke begrenzen wenn zu hoch
  float amp = vOut / CoilOhm; //Frage: Ist das dann die Stromstärke die aus den Akkus gezogen wird? Ich denk schon
  if(amp > AMP_MAX)
  {
    //Display: Ampere too high
    vOut = CoilOhm * AMP_MAX;
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
{ //https://www.electroschematics.com/9351/arduino-digital-voltmeter/
  
  //Für höhere genauigkeit müssen wir Ohm der Widerstände 1x ausmessen und anpassen
  float R1 = 100000.f;
  float R2 = 10000.f;

  //Gesamt Voltage berechnen
  int value = analogRead(VBAT_IN);
  float vout = (value * 5.0) / 1024.0; //Für höhere genauigkeit müssen wir die Arduino 5v Spannung messen und anpassen (bei 5.0)
  float vBat = vout / ( R2 / (R1+R2) );

  if(A != NULL && B != NULL) //Beide Zellen berechnen
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
  if(state != lastState && state == HIGH)
  {
    if(digitalRead(RE_DT) != state) //CCW
      ret = 2;
    else  //CW
      ret = 1;
  }
  lastState = state;
  return ret;
}

bool query(String question)
{
  // YES/TRUE - NO/FALSE Abfrage über Drehgeber
  //CCW = Y/T | CW = N/F
  
  //Display: Question
  Serial.println(question);
  Serial.println("<-- YES | NO -->");
  byte tmp = 0;
  byte r = 0;
  while(r == 0 || digitalRead(RE_BUTTON) == LOW)
  {
    tmp = rotaryEncoder();
    if(tmp != 0)
      r = tmp;
  }
  
  return (r==2) ? true : false;
}

