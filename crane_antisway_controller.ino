#include <Arduino.h>
#include <SoftwareSerial.h>

// Talk to the Nano on pins A4 (RX) and A5 (TX)
// Wire: Uno A5 --> Nano pin 10
SoftwareSerial nanoLine(A4, A5);

// Max PWM speed allowed for each motor (0-255)
const int LIMIT_TROLLEY = 255;
const int LIMIT_BODY    = 130;  // Body rotation is limited to avoid mechanical stress
const int LIMIT_HOOK    = 255;

// Motor pins on the L298N driver
const int bodyPin1   = 11;
const int bodyPin2   = 12;
const int bodyPWM    = 3;

const int hookPin1   = 8;
const int hookPin2   = 9;
const int hookPWM    = 6;

const int trolleyPin1 = 5;
const int trolleyPin2 = 4;
const int trolleyPWM  = 10;

// Joystick analog input pins
const int joyTrolleyPin = A0;
const int joyHookPin    = A1;
const int joyBodyPin    = A2;

// Ignore joystick movements smaller than this to avoid motor jitter
const int deadZone = 80;

// Tracks whether Python is currently running a timed motor command
bool          aiActive   = false;
unsigned long aiTimer    = 0;
unsigned long aiDuration = 0;

// Serial receive buffer
const byte numChars = 64;
char       receivedChars[numChars];
boolean    newData = false;


void setup() {
  Serial.begin(115200);
  nanoLine.begin(9600);

  pinMode(bodyPin1,    OUTPUT);
  pinMode(bodyPin2,    OUTPUT);
  pinMode(bodyPWM,     OUTPUT);
  pinMode(hookPin1,    OUTPUT);
  pinMode(hookPin2,    OUTPUT);
  pinMode(hookPWM,     OUTPUT);
  pinMode(trolleyPin1, OUTPUT);
  pinMode(trolleyPin2, OUTPUT);
  pinMode(trolleyPWM,  OUTPUT);

  Serial.println("<Ready>");
}


void loop() {
  recvWithStartEndMarkers();

  if (newData) {
    parseData();
    newData = false;
  }

  if (aiActive) {
    // Stop motors once the requested duration has passed
    if (millis() - aiTimer >= aiDuration) {
      stopAll();
      aiActive = false;
      Serial.println("<DONE>");
    }
  } else {
    // No Python command running — hand control back to the joystick
    readJoystick();
  }
}


void readJoystick() {
  // Center joystick reads at 512 on a 10-bit ADC, so subtract to get -512..+512
  int joyBody    = analogRead(joyBodyPin)    - 512;
  int joyHook    = analogRead(joyHookPin)    - 512;
  int joyTrolley = analogRead(joyTrolleyPin) - 512;

  // Map joystick deflection to motor speed, zero if inside the deadzone
  int speedBody = abs(joyBody) > deadZone ? map(abs(joyBody), 0, 512, 0, LIMIT_BODY) : 0;
  if (joyBody < 0) speedBody = -speedBody;

  int speedHook = abs(joyHook) > deadZone ? map(abs(joyHook), 0, 512, 0, LIMIT_HOOK) : 0;
  if (joyHook > 0) speedHook = -speedHook;  // Invert so joystick up = hook up

  int speedTrolley = abs(joyTrolley) > deadZone ? map(abs(joyTrolley), 0, 512, 0, LIMIT_TROLLEY) : 0;
  if (joyTrolley > 0) speedTrolley = -speedTrolley;

  driveMotor(bodyPin1,    bodyPin2,    bodyPWM,    speedBody);
  driveMotor(hookPin1,    hookPin2,    hookPWM,    speedHook);
  driveMotor(trolleyPin1, trolleyPin2, trolleyPWM, speedTrolley);
}


void parseData() {
  // Commands arrive as comma-separated text between < and > markers
  // Example movement command:  <J,0,0,200,1500>
  // Example magnet command:    <M,1>
  // Example stop command:      <S>

  char *token = strtok(receivedChars, ",");

  if (strcmp(token, "J") == 0) {
    int  rot      = atoi(strtok(NULL, ","));
    int  hook     = atoi(strtok(NULL, ","));
    int  trol     = atoi(strtok(NULL, ","));
    long duration = atol(strtok(NULL, ","));

    // Clamp values so Python can never exceed the hardware safety limits
    driveMotor(bodyPin1,    bodyPin2,    bodyPWM,    constrain(rot,  -LIMIT_BODY,    LIMIT_BODY));
    driveMotor(hookPin1,    hookPin2,    hookPWM,    constrain(hook, -LIMIT_HOOK,    LIMIT_HOOK));
    driveMotor(trolleyPin1, trolleyPin2, trolleyPWM, constrain(trol, -LIMIT_TROLLEY, LIMIT_TROLLEY));

    aiActive   = true;
    aiTimer    = millis();
    aiDuration = duration;
    Serial.println("<ACK>");

  } else if (strcmp(token, "M") == 0) {
    int state = atoi(strtok(NULL, ","));

    if (state == 1) {
      nanoLine.print('1');
      Serial.println("<MAGNET_ON>");
    } else {
      nanoLine.print('0');
      Serial.println("<MAGNET_OFF>");
    }

  } else if (strcmp(token, "S") == 0 || strcmp(token, "s") == 0) {
    stopAll();
    aiActive = false;
    Serial.println("<STOPPED>");
  }
}


// Drives one motor on an L298N:  positive speed = forward, negative = backward, 0 = stop
void driveMotor(int p1, int p2, int pwmPin, int speed) {
  if (speed == 0) {
    digitalWrite(p1, LOW);
    digitalWrite(p2, LOW);
    analogWrite(pwmPin, 0);
  } else if (speed > 0) {
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
    analogWrite(pwmPin, speed);
  } else {
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
    analogWrite(pwmPin, abs(speed));
  }
}


void stopAll() {
  driveMotor(bodyPin1,    bodyPin2,    bodyPWM,    0);
  driveMotor(hookPin1,    hookPin2,    hookPWM,    0);
  driveMotor(trolleyPin1, trolleyPin2, trolleyPWM, 0);
  nanoLine.print('0');  // Tell the Nano to release the magnet
}


// Reads the serial port one byte at a time, looking for <message> format
// This approach never misses a byte even at high baud rates
void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte    ndx = 0;
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress) {
      if (rc != '>') {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) ndx = numChars - 1;  // Prevent buffer overflow
      } else {
        receivedChars[ndx] = '\0';  // Null-terminate the string
        recvInProgress = false;
        newData = true;
        ndx = 0;
      }
    } else if (rc == '<') {
      recvInProgress = true;
    }
  }
}
