// STI Las Pinas - v1
// Testing of Components/Wirings
// version 3.0 - With Infrared and LCD usability.

#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

VR myVR(52, 53);   // 2:RX 3:TX, you can choose your favourite pins.

uint8_t records[7]; // save record
uint8_t buf[64];

boolean forwardVR = false;
boolean backwardVR = false;
boolean rightVR = false;
boolean leftVR = false;
boolean stopVR = false;

#define test1   (0) // FORWARD
#define test2   (1) // BACKWARD
#define test3   (2) // RIGHT
#define test4   (3) // LEFT
#define test5   (4) // STOP

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

const int buzzer = 13;

const int speedA = 4;
const int directionA = 5;

const int speedB = 8;
const int directionB = 9;

const int joystickBack = 28;
const int joystickRight = 29;
const int joystickFront = 30;
const int joystickLeft = 31;

int jBVal;
int jRVal;
int jFVal;
int jLVal;

boolean manualOverride = false;

const int irBack = 22;
const int irRight = 23;
const int irLeft = 24;

int irBVal;
int irRVal;
int irLVal;


int pwmA = 255;
int pwmB = 255;
int pwmStop = 0;


void setup() {
  Serial.begin(9600);
  myVR.begin(9600);
  Serial.println(F("START"));

  lcd.begin(20, 4);
  lcd.backlight();

  pinMode(buzzer, OUTPUT);

  pinMode(speedA, OUTPUT);
  pinMode(directionA, OUTPUT);

  pinMode(speedB, OUTPUT);
  pinMode(directionB, OUTPUT);

  pinMode(joystickBack, INPUT_PULLUP);
  pinMode(joystickRight, INPUT_PULLUP);
  pinMode(joystickFront, INPUT_PULLUP);
  pinMode(joystickLeft, INPUT_PULLUP);

  pinMode(irBack, INPUT_PULLUP);
  pinMode(irLeft, INPUT_PULLUP);
  pinMode(irRight, INPUT_PULLUP);

  VRmoduleLoad();
  Stop();
}

void loop() {
  joystickModule();
  irBeeping();
  irSensor();
  VRModuleTest();

  if (manualOverride == false) {
    VRDecide();
  }

  LCDupdate();
}


void LCDupdate() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Voice Controlled");
  lcd.setCursor(0, 1);
  lcd.print("Wheelchair");
  lcd.setCursor(0, 2);
  lcd.print("Speed Right: ");
  lcd.print(pwmA);
  lcd.setCursor(0, 3);
  lcd.print("Speed Left: ");
  lcd.print(pwmB);
}

void VRDecide() {
  irStopping();

  if (forwardVR == true) {
    forward();
    Serial.println("F");
  } else if (backwardVR == true) {
    backward();
    Serial.println("B");
  } else if (rightVR == true) {
    clockwise();
    Serial.println("R");
  } else if (leftVR == true) {
    Serial.println("L");
    counterclockwise();
  }
}

void VRModuleTest() {
  int ret;
  ret = myVR.recognize(buf, 50);
  if (ret > 0) {
    // to reset booleans every call of VRModuleTest
    manualOverride = false;
    forwardVR = false;
    backwardVR = false;
    rightVR = false;
    leftVR = false;
    stopVR = false;
    switch (buf[1]) {
      case test1:
        Serial.println(F("FORWARD"));
        backwardVR = true;
        Stop();
        beep();
        break;
      case test2:
        Serial.println(F("BACKWARD"));
        forwardVR = true;
        Stop();
        beep();
        break;
      case test3:
        Serial.println(F("RIGHT"));
        rightVR = true;
        Stop();
        beep();
        break;
      case test4:
        Serial.println(F("LEFT"));
        leftVR = true;
        Stop();
        beep();
        break;
      case test5:
        Serial.println(F("STOP"));
        stopVR = true;
        beep();
        break;
      default:
        Serial.println("Record function undefined");
        break;
    }
    /** voice recognized */
    printVR(buf);
  }
}

void beep() {
  digitalWrite(buzzer, HIGH);
  delay(375);
  digitalWrite(buzzer, LOW);
  delay(375);
  digitalWrite(buzzer, HIGH);
  delay(375);
  digitalWrite(buzzer, LOW);
  delay(375);
}

void VRmoduleLoad() {
  if (myVR.clear() == 0) {
    Serial.println("Recognizer cleared.");
  } else {
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while (1);
  }

  if (myVR.load((uint8_t)test1) >= 0) {
    Serial.println("Loading forward...");
  }

  if (myVR.load((uint8_t)test2) >= 0) {
    Serial.println("Loading backward...");
  }

  if (myVR.load((uint8_t)test3) >= 0) {
    Serial.println("Loading right...");
  }

  if (myVR.load((uint8_t)test4) >= 0) {
    Serial.println("Loading left...");
  }

  if (myVR.load((uint8_t)test5) >= 0) {
    Serial.println("Loading stop...");
  }
}

void irSensor() {
  irBVal = digitalRead(irBack);
  irRVal = digitalRead(irRight);
  irLVal = digitalRead(irLeft);

  if (irRVal == LOW) {
    Serial.println(F("Right IR Activated"));
  }

  if (irBVal == LOW) {
    Serial.println(F("Back IR Activated"));
  }

  if (irLVal == LOW) {
    Serial.println(F("Left IR Activated"));
  }

}

void irStopping() {
  irSensor();
  if (irRVal == LOW || irBVal == LOW || irLVal == LOW) {
    Serial.println(F("Voice STOPPED by IR!"));
    manualOverride = true;
    Stop();
  }
}


void irBeeping() {
  while (jBVal == LOW || jRVal == LOW || jFVal == LOW || jLVal == LOW) {
    jBVal = digitalRead(joystickBack);
    jRVal = digitalRead(joystickRight);
    jFVal = digitalRead(joystickFront);
    jLVal = digitalRead(joystickLeft);

    digitalWrite(buzzer, HIGH);
    delay(60);
    digitalWrite(buzzer, LOW);
    delay(60);
  }
}

void joystickModule() {
  jBVal = digitalRead(joystickBack);
  jRVal = digitalRead(joystickRight);
  jFVal = digitalRead(joystickFront);
  jLVal = digitalRead(joystickLeft);

  if (jBVal == LOW) {
    Serial.println(F("FRONT"));
    manualOverride = true;
    forward();

  } else if (jRVal == LOW) {
    Serial.println(F("RIGHT"));
    manualOverride = true;
    clockwise();

  } else if (jFVal == LOW) {
    Serial.println(F("BACK"));
    manualOverride = true;
    backward();

  } else if (jLVal == LOW) {
    Serial.println(F("LEFT"));
    manualOverride = true;
    counterclockwise();

  } else if (jBVal == HIGH && jRVal == HIGH && jFVal == HIGH && jLVal == HIGH && manualOverride == true) {
    Stop();
  }

}

void forward() {
  digitalWrite(directionA, HIGH);
  digitalWrite(directionB, LOW);

  runMotor();
}

void backward() {
  digitalWrite(directionA, LOW);
  digitalWrite(directionB, HIGH);

  runMotor();
}

void clockwise() {
  digitalWrite(directionA, HIGH);
  digitalWrite(directionB, HIGH);

  runMotor();
}

void counterclockwise() {
  digitalWrite(directionA, LOW);
  digitalWrite(directionB, LOW);

  runMotor();
}

void Stop() {
  analogWrite(speedA, pwmStop);
  analogWrite(speedB, pwmStop);
}

void runMotor() {
  analogWrite(speedA, pwmA);
  analogWrite(speedB, pwmB);
}

void printSignature(uint8_t *buf, int len)
{
  int i;
  for (i = 0; i < len; i++) {
    if (buf[i] > 0x19 && buf[i] < 0x7F) {
      Serial.write(buf[i]);
    }
    else {
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}


void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if (buf[0] == 0xFF) {
    Serial.print("NONE");
  }
  else if (buf[0] & 0x80) {
    Serial.print("UG ");
    Serial.print(buf[0] & (~0x80), DEC);
  }
  else {
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if (buf[3] > 0) {
    printSignature(buf + 4, buf[3]);
  }
  else {
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}
