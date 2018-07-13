//Basic code für Vape
//OHNE Display, OHNE Rotary Encoder, OHNE Ohmmeter für Coil


//Pinbelegung
#define FIREBUTTON  2   //Feuerknopf | Digital Input
#define VBAT_IN     A0  //Battery Voltage | Analog Input
#define VMB_IN      A1  //Voltage zwischen den beiden Zellen, zur berechnung von jeweiligem Akkustand
#define GATE_PIN    8   //Mosfet Gate | PWM Output
#define RE_CLK      3   //Drehgeber Clock
#define RE_DT       4   //Drehgeber Data
#define RE_BUTTON   5   //Drehgeber Button

//Vape Begrenzungen
#define MinWatt 10
#define MaxWatt 150
#define MinOhm  0.1
#define MaxOhm  2.0
#define MaxAmp  35

//Globale Variablen
int Watt = 50;
float CoilOhm = 0.2f;

//Funktionsprototypen
void rotaryEncoder();
float getVBat (float *A = NULL, float *B = NULL);
void fire();


void setup()
{
  pinMode(FIREBUTTON, INPUT);
  pinMode(VBAT_IN, INPUT);
  pinMode(VMB_IN, INPUT);
  pinMode(GATE_PIN, OUTPUT);
  
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
    
  
  float vBat, vBatA, vBatB;
  vBat = getVBat(&vBatA,&vBatB);
  if(vBatA < 3.4 || vBatB < 3.4)
  {
    //Display: Battery Low
    Serial.print("Battery Low | A: "); Serial.print(vBatA); Serial.print("V, B: "); Serial.print(vBatB); Serial.print("V\n");
    return;
  }

  if(Watt<MinWatt)
    Watt=MinWatt;
  if(Watt>MaxWatt)
    Watt=MaxWatt;
  if(CoilOhm < MinOhm)
  {
    //Display: Ohm too low
    Serial.print("Ohm 2low: "); Serial.print(CoilOhm); Serial.print("Ω\n");
    return;
  }
  if(CoilOhm > MaxOhm)
  {
    Serial.print("Ohm 2high: "); Serial.print(CoilOhm); Serial.print("Ω\n");
    return;
  }

  if(digitalRead(FIREBUTTON) == HIGH)
  {
    //Für Tastensperre
    if(millis() - tLastFire <= 250)
      counter++;
    else
      counter=0;
    tLastFire = millis();

    //Vaperino
    while(digitalRead(FIREBUTTON) == HIGH && !bLock) //AUFPASSEN: Öffner oder Schließer  
    {
      fire();
    }
    analogWrite(GATE_PIN, 0);
  }
  
  
}

void fire()
{  
  float vBat = getVBat();  
  float vOut = sqrt( Watt * CoilOhm ); //Ohmsches Gesetz U=√P*R

  float amp = vOut / CoilOhm;
  if(amp > MaxAmp)
  {
    //Display: Ampere too high
    vOut = CoilOhm * MaxAmp;
  }
  
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
  float R1 = 100000.f;
  float R2 = 10000.f;

  //Gesamt Voltage berechnen
  int value = analogRead(VBAT_IN);
  float vout = (value * 5.0) / 1024.0;
  float vBat = vout / ( R2 / (R1+R2) );

  if(A != NULL && B!= NULL) //Beide Zellen berechnen
  {
    value = analogRead(VMB_IN);
    vout = (value * 5.0) / 1024.0;
    float vA = vout / ( R2 / (R1+R2) );
    float vB = vBat - vA;
    //In die Pointer schreiben
    *A = vA;
    *B = vB;
  }

  return vBat;
}

void rotaryEncoder()
{
  
}

