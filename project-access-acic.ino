#include "Wire.h"
#include <stdlib.h>

// Controller identification
const int controller = 0;

// Types of messages
const int RED = 0;
const int GREEN = 1;
const int OFF = 2;
const int PING = 3;
const int ACK = 4;
const int STATUS = 5;

// Jumpers pins
const byte jumper2 = 4;
const byte jumper1 = 3;
const byte jumper0 = 2; 

// Traffic lights
#define NUMBER_OF_TRAFFIC_LIGHTS 8
#define TL_AR 10
#define TL_AY 9
#define TL_AG 8
#define PL_AR 7
#define PL_AG 6
#define PEDESTRIAN_BUTTON 5
#define TL_BR 13
#define TL_BY 12
#define TL_BG 11
int lights[NUMBER_OF_TRAFFIC_LIGHTS] = { TL_BR, TL_BY, TL_BG, TL_AR, TL_AY, TL_AG, PL_AR, PL_AG };

// Period of time in ms to stay yellow during trafffic cycle: red -> yellow -> green
const int yellowInterval = 500;

// Variables to handle the bliking yellow leds from TL_A and TL_B
unsigned long previousMillisYellowBlink = 0; // Will store last time LED was updated
int yellowBlinkInterval = 1000;
int yellowLedState = LOW;
bool blinkYellow = false;

// Information variables
int pedestRedFailing = 0;
int pedestYellowFailing = 0;
int pedestGreenFailing = 0;
int redFailing = 0;
int yellowFailing = 0;
int greenFailing = 0;
int timerActivated = 0;

// Variables to handle Pedestrian Button
int lastPedestrianButtonState;
int currentPedestrianButtonState;

// Variables to handle ack and status messages
char ack[4];
char status[5];

// Flags to handle received messages by Junction
bool msgReceived = false; // True: when a message is received; False: after calling the method that handles the message
int msgType = -1; // Gets the identifier of message received: {RED, GREEN, OFF, PING}

// Setup
void setup() {
  // Starts serial for output
  Serial.begin(9600);

  // Initializes the pedestrian button as an input
  pinMode(PEDESTRIAN_BUTTON, INPUT);

  // Initializes the jumpers as inputs
  pinMode(jumper2, INPUT);
  pinMode(jumper1, INPUT);
  pinMode(jumper0, INPUT);

  // join i2c bus with address #entryNumber
  int entryNumber = getEntryNumber();
  Serial.print("ENTRY ");
  Serial.println(entryNumber);
  Wire.begin(entryNumber);

  // when master sends a message
  Wire.onReceive(receiveEvent);

  //when master asks for the answer
  Wire.onRequest(requestEvent);

  // Initializes the traffic lights as outputs
  for (int i = 0; i < NUMBER_OF_TRAFFIC_LIGHTS; i++)
    pinMode(lights[i], OUTPUT);
}

void loop() {

  checkPedestrianButton();
  
  if (blinkYellow) handleYellowBlink();

  if(msgReceived) {
    switch (msgType) {
      case RED: receiveRED(); msgReceived = false; break;
      case GREEN: receiveGREEN(); msgReceived = false; break;
      case OFF: receiveOFF(); msgReceived = false; break;
      case PING: msgReceived = false; break;
      default: Serial.println("Should never happen, but just in case...");
    }
  }
}

// The identification of the roundabout entry corresponding to the traffic light is configured by jumpers (or fixed wires) connected to input ports of the Arduino controller – 1, 2, 3, 4 – in ascending order anti-clockwise. ?????? ARDUINO CONTROLLER ?????
int getEntryNumber() {
  char entry[3];
  if (digitalRead(jumper2) == HIGH)
    entry[0] = '1';
  else
    entry[0] = '0';
  if (digitalRead(jumper1) == HIGH)
    entry[1] = '1';
  else
    entry[1] = '0';
  if (digitalRead(jumper0) == HIGH)
    entry[2] = '1';
  else
    entry[2] = '0';

  int number = strtol(entry, NULL, 2);
  return number;
}

void checkPedestrianButton() {
  lastPedestrianButtonState = currentPedestrianButtonState;      // Stores the previous state of the push button
  currentPedestrianButtonState = digitalRead(PEDESTRIAN_BUTTON);  // Stores the present state of the push button
  if (lastPedestrianButtonState == HIGH && currentPedestrianButtonState == LOW) {
    timerActivated = 1;
    Serial.println("FUI CARREGADO!!!!!!!!");
  }
}

// Triggered when the Access gets interrupted to read the message from the Controller
void receiveEvent(int howMany) {
  if (howMany == 4) {  // Extra security check (how many bytes were sent)
    char sender = Wire.read();
    char operationNumber = Wire.read();
    char destination = Wire.read();
    char integrityByte = Wire.read(); // TODO: CHECK IF MESSAGE IS VALID ACCORDINGLY TO THE PROTOCOL
    if ((int)destination == getEntryNumber()) {
      Serial.print("RECEBI ");
      Serial.print((int)operationNumber);
      Serial.print(" para ");
      Serial.println((int)destination);
      switch (sender) {
        case controller:
          switch ((int)operationNumber) {
            case RED: msgReceived = true; msgType = RED; break;
            case GREEN: msgReceived = true; msgType = GREEN; break;
            case OFF: msgReceived = true; msgType = OFF; break;
            case PING: msgReceived = true; msgType = PING; break;
            default: Serial.println("Should never happen, but just in case...");
          }
          break;
        default: Serial.println("Should never happen, but just in case...");
      }
    }
  }
}

// The coordination between TL kA and TL kB must be self-sufficient, implemented locally and not by the roundabout controller
void receiveRED() {
  Serial.print("RED INICIO: ");
  Serial.println(millis());

  // Stop blinking YELLOW (turned ON when OFF was received)
  blinkYellow = false;

  // Access light turns YELLOW
  digitalWrite(TL_AG, LOW);
  digitalWrite(TL_AY, HIGH);

  // Each passage through Yellow will take 0,5 seconds
  // Each passage through Yellow will take 0,5 seconds
  delayMilliseconds(yellowInterval);

  // Access light turns RED
  digitalWrite(TL_AY, LOW);
  digitalWrite(TL_AR, HIGH);

  // Pedestrian light turns GREEN
  digitalWrite(PL_AG, HIGH);
  digitalWrite(PL_AR, LOW);
  // Resets timer since the pedestrian light is now green
  timerActivated = 0;

  // Roundabout light turns YELLOW
  digitalWrite(TL_BR, LOW);
  digitalWrite(TL_BY, HIGH);

  // Each passage through Yellow will take 0,5 seconds
  delayMilliseconds(yellowInterval);

  // Roundabout light turns GREEN
  digitalWrite(TL_BY, LOW);
  digitalWrite(TL_BG, HIGH);

  Serial.print("RED FIM: ");
  Serial.println(millis());
}

// The coordination between TL kA and TL kB must be self-sufficient, implemented locally and not by the roundabout controller
void receiveGREEN() {
  Serial.print("GREEN INICIO: ");
  Serial.println(millis());

  // Stop blinking YELLOW (turned ON when OFF was received)
  blinkYellow = false;

  // Pedestrian light turns RED
  digitalWrite(PL_AG, LOW);
  digitalWrite(PL_AR, HIGH);

  // Roundabout light turns YELLOW
  digitalWrite(TL_BG, LOW);
  digitalWrite(TL_BY, HIGH);

  // Each passage through Yellow will take 0,5 seconds
  delayMillisecondsPedestrian(yellowInterval);

  // Roundabout light turns RED
  digitalWrite(TL_BY, LOW);
  digitalWrite(TL_BR, HIGH);

  // Access light turns YELLOW
  digitalWrite(TL_AR, LOW);
  digitalWrite(TL_AY, HIGH);

  // Each passage through Yellow will take 0,5 seconds
  delayMillisecondsPedestrian(yellowInterval);

  // Access light turns GREEN
  digitalWrite(TL_AY, LOW);
  digitalWrite(TL_AG, HIGH);

  Serial.print("GREEN FIM: ");
  Serial.println(millis());
}

void delayMilliseconds(unsigned long milliseconds) {
  unsigned long startTime = millis();
  while (millis() - startTime < milliseconds) {
    // Do nothing
  }
}

void delayMillisecondsPedestrian(unsigned long milliseconds) {
  unsigned long startTime = millis();
  while (millis() - startTime < milliseconds) {
    checkPedestrianButton();
  }
}

void receiveOFF() {
  digitalWrite(PL_AG, LOW);
  digitalWrite(PL_AR, LOW);
  digitalWrite(TL_AG, LOW);
  digitalWrite(TL_AR, LOW);
  digitalWrite(TL_BG, LOW);
  digitalWrite(TL_BR, LOW);
  blinkYellow = true;
}

void handleYellowBlink() {
  if (millis() - previousMillisYellowBlink > yellowBlinkInterval) {
    // If the LED is OFF turn it ON and vice-versa:
    if (yellowLedState == LOW) {
      yellowLedState = HIGH;
    } else {
      yellowLedState = LOW;
    }
    // Sets the LEDs with the ledState (HIGH/LOW):
    digitalWrite(TL_AY, yellowLedState);
    digitalWrite(TL_BY, yellowLedState);
    previousMillisYellowBlink = millis();
  }
}

// The ACK (x) should be send as response to RED(x), GREEN(x) and OFF(x) requests.
void setACK() {
  ack[0] = (char)getEntryNumber();
  ack[1] = (char)ACK;
  ack[2] = (char)controller;
  ack[3] = (char)(getEntryNumber() + ACK + controller);
}

// Status(X) will be the response from the traffic light when the controller do a Ping(x) request.
void setSTATUS() {
  char data[] = { pedestRedFailing + '0', pedestYellowFailing + '0', pedestGreenFailing + '0', redFailing + '0', yellowFailing + '0', greenFailing + '0', timerActivated + '0', '0' };
  int information = strtol(data, NULL, 2);
  status[0] = (char)getEntryNumber();
  status[1] = (char)STATUS;
  status[2] = (char)controller;
  status[3] = (char)information;
  status[4] = (char)(getEntryNumber() + information + STATUS + controller);
}

void requestEvent(int howMany) {
  if (msgType == RED || msgType == GREEN || msgType == OFF) {
    setACK();
    for (int i = 0; i < ACK; i++)
      Wire.write(ack[i]);
  }
  if (msgType == PING) {
    setSTATUS();
    Serial.print("Timer Activated: ");
    Serial.println(timerActivated);
    for (int i = 0; i < STATUS; i++)
      Wire.write(status[i]);
  }
}