#include <EEPROM.h>

// Pinbelegung
#define FIREBUTTON  2   //Feuerknopf | Digital Input
#define VBAT_IN     A0  //Battery Voltage | Analog Input
#define VMB_IN      A1  //Voltage zwischen den beiden Zellen, zur berechnung von jeweiligem Akkustand
#define GATE_PIN    9   //Mosfet Gate | PWM Output
#define RE_CLK      3   //Drehgeber Clock
#define RE_DT       4   //Drehgeber Data
#define RE_BUTTON   5   //Drehgeber Button

// Vape Begrenzungen
#define MULTIPRESS_DELAY  300
#define LONGPRESS_TIME    600
#define WATT_MIN          10
#define WATT_MAX          150
#define OHM_MIN           0.1
#define OHM_MAX           2.0
#define AMP_MAX           35
#define DEBOUNCE_MS       150

// Voltage Reading
#define RES_NET_1   99500
#define RES_NET_2   10020

// EEPROM Adressen
#define ADR_WATT  0 //int
#define ADR_PUFFS 4 //int
#define ADR_OHM   8 //float

unsigned int  watt,
              multiPressCounter = 1,
              gateDutyCycle,
              mode; // 0 = Sperre; 1 = Vapen; 2 = Einstellungen

unsigned long fireTimePressed,
              fireTimeReleased,
              fireTimeDelta,
              fireTimeGap,
              lastFireTime;

bool  fireButtonState,
      fireState,
      batteryLow,
      clockCurrent,
      clockLast,
      dataCurrent,
      dataLast,
      stateManaged,
      currentStatePrinted;

float batteryVoltage;


// Funktionsprototypen
void initializeButtons();
void fireButtonPressed();
void serialButtonPressInfo();
void debugInit();

void setMode(int pMode);
int getMode();

int readWatt();
void writeWatt(int pWatt);

float readVoltage();

// Programm
void setup() {
  Serial.begin(9600);
  Serial.println("Esketit");
  debugInit();
  initializeButtons();
  initializeInterrupt();
  setPwmFrequency();
}

void loop() {
  if (fireButtonState == true && mode == 1) {
    fireState = true;
    manageFire();
    stateManaged = true;
  } else {
    fireState = false;
    manageFire();
    stateManaged = true;
  }

  if (currentStatePrinted == false) {
    switch (mode) {
      case 0:
        Serial.println("napeVation is Locked.");
        break;
      case 1:
        Serial.println("napeVation is unlocked.");
        break;
      case 2:
        Serial.println("napeVation Menu");
        break;
      default:
        Serial.println("WTF");
    }
    currentStatePrinted = true;
  }

  checkForEncoderRotation();
}

// Funktionen
void initializeButtons() {
  pinMode(FIREBUTTON, INPUT);
  pinMode(VBAT_IN, INPUT);
  pinMode(VMB_IN, INPUT);
  pinMode(GATE_PIN, OUTPUT);
  pinMode(RE_CLK, INPUT);
  pinMode(RE_DT, INPUT);
  pinMode(RE_BUTTON, INPUT);
}

void debugInit() {
  gateDutyCycle = 180;
}

void initializeInterrupt() {
  attachInterrupt(digitalPinToInterrupt(FIREBUTTON), fireButtonChange, CHANGE);
}

void fireButtonChange() {
  fireButtonState = digitalRead(FIREBUTTON);

  // Warte ob die Änderung nach bestimmter Zeit immernoch da ist
  // Wenn ja, Tastendruck
  // Wenn nein, Bounced der Butonneon
  delay(DEBOUNCE_MS);

  if (fireButtonState == digitalRead(FIREBUTTON)) {
    if (fireButtonState == HIGH) {
      // FIREBUTTON PRESSED ///////
      fireButtonPressed();
    } else if (fireButtonState == LOW) {
      // FIREBUTTON RELEASED //////
      fireButtonReleased();
    }
  }
}

void manageFire() {
  if (fireState == true) {
    analogWrite(GATE_PIN, gateDutyCycle);
    // Serial.print("Duty cycle: "); Serial.print(gateDutyCycle); Serial.print("\n");
    stateManaged = true;
  } else {
    analogWrite(GATE_PIN, 0);
    // Serial.println("Stopped Firing!");
    stateManaged = true;
  }
}

void fireButtonPressed() {
  fireTimePressed = millis();
  fireTimeGap = fireTimePressed - fireTimeReleased;

  if (fireTimeGap <= MULTIPRESS_DELAY) {
    multiPressCounter++;
  } else {
    multiPressCounter = 1;
  }

  fireButtonState = true;
}

void fireButtonTripplePress() {
  switch (mode) {
    case 0:
      mode = 1;
      break;
    case 1:
      mode = 2;
      break;
    case 2:
      mode = 0;
      break;
    default:
      Serial.println("WTF");
  }
  currentStatePrinted = false;
}

void fireButtonLongPress() {

}

void setPwmFrequency() {
//TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
}

void fireButtonReleased() {
  fireTimeReleased = millis();
  fireTimeDelta = fireTimeReleased - fireTimePressed;

  serialButtonPressInfo();

  if (fireTimeDelta >= LONGPRESS_TIME) {
    fireButtonLongPress();
  }

  if (multiPressCounter == 3) {
    fireButtonTripplePress();
  }

  lastFireTime = millis();

  fireButtonState = false;
}

void checkForEncoderRotation() {
  clockCurrent = digitalRead(RE_CLK);

  if (clockCurrent != clockLast && clockCurrent == true) {
    if (digitalRead(RE_DT) != clockCurrent) {
      // Gegen Uhrzeigersinn
      encoderRotation(false);
    } else {
      // Im Uhrzeigersinn
      encoderRotation(true);
    }
  }

  clockLast = digitalRead(RE_CLK);
}

void encoderRotation(bool pDirection) { // true = clockwise
  switch (mode) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      if (pDirection == true) {
        gateDutyCycle = gateDutyCycle >= 255 ? 255 : gateDutyCycle + 5;
      } else if (pDirection == false) {
        gateDutyCycle = gateDutyCycle <=   0 ?   0 : gateDutyCycle - 5;
      }
      Serial.println(gateDutyCycle);
      break;
    default:
      Serial.println("WTF");
      break;
  }
}

void serialButtonPressInfo() {
  Serial.print("<< Firebutton was pressed for ");
  Serial.print(fireTimeDelta);
  Serial.print("ms. (");
  Serial.print(multiPressCounter);
  Serial.print(") [");
  Serial.print(fireTimeGap);
  Serial.println("ms delay] >>");
}

void setMode(int pMode) {
  if (pMode >= 0 && pMode <= 2) {
    mode = pMode;
    Serial.print("Mode changed to: ");
    Serial.println(pMode);
  } else {
    Serial.println("Invalid mode selected");
  }
}

int getMode() {
  return mode;
}

float readVoltage() {
  int analogVoltage = analogRead(VMB_IN);
  float inputVoltage = float(analogVoltage * 4.96f / 1024.0f);

  if (inputVoltage < 0.1f) {
    inputVoltage = 0.0f;
  }

  return inputVoltage;
}
