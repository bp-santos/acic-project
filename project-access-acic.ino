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

  // Initializes the jumpers as inputs
  pinMode(jumper1, INPUT);
  pinMode(jumper0, INPUT);

  // join i2c bus with address #entryNumber
  int entryNumber = getEntryNumber();
  Wire.begin(entryNumber);

  Wire.onReceive(receiveEvent); // when master sends a message
  Wire.onRequest(requestEvent); //when master asks for the answer

  // Initializes the traffic lights as outputs
  for (int i = 0; i < NUMBER_OF_TRAFFIC_LIGHTS; i++)
    pinMode(lights[i], OUTPUT);

  // Initializes the pedestrian button as an input
  pinMode(PEDESTRIAN_BUTTON, INPUT);
}

// Loop
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

// Gets the roundabout entry based on 2 jumpers
int getEntryNumber() {
  char entry[2];
  if (digitalRead(jumper1) == HIGH)
    entry[0] = '1';
  else
    entry[0] = '0';
  if (digitalRead(jumper0) == HIGH)
    entry[1] = '1';
  else
    entry[1] = '0';

  int number = strtol(entry, NULL, 2) ;
  return number + 1;
}

// Checks if the pedestrian button was pressed
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

// Turns off a given led pin and turns on the other given led pin
void changeLight(int turnOff, int turnOn) {
  digitalWrite(turnOff, LOW);
  digitalWrite(turnOn, HIGH);
}

// Handles the reception of a RED message by doing the sequence of traffic lights
void receiveRED() {
  blinkYellow = false;
  changeLight(TL_AG, TL_AY); // Access light turns YELLOW
  delayMilliseconds(yellowInterval); // Passage through yellow
  changeLight(TL_AY, TL_AR); // Access light turns RED
  changeLight(PL_AR , PL_AG); // Pedestrian light turns GREEN
  timerActivated = 0; // Resets timer since the pedestrian light is now green
  changeLight(TL_BR, TL_BY); // Roundabout light turns YELLOW
  delayMilliseconds(yellowInterval); // Passage through yellow
  changeLight(TL_BY, TL_BG); // Roundabout light turns GREEN
}

// Handles the reception of a GREEN message by doing the sequence of traffic lights
void receiveGREEN() {
  blinkYellow = false;
  changeLight(TL_BG, TL_BY); // Roundabout light turns YELLOW
  delayMillisecondsPedestrian(yellowInterval); // Passage through yellow
  changeLight(TL_BY, TL_BR); // Roundabout light turns RED
  changeLight(PL_AG, PL_AR); // Pedestrian light turns RED
  changeLight(TL_AR, TL_AY); // Access light turns YELLOW
  delayMillisecondsPedestrian(yellowInterval); // Passage through yellow
  changeLight(TL_AY, TL_AG); // Access light turns GREEN
}

// Delay with milliseconds
void delayMilliseconds(unsigned long milliseconds) {
  unsigned long startTime = millis();
  while (millis() - startTime < milliseconds) {
    // Do nothing
  }
}

// Delay with milliseconds while checking if the pedestrian button was pressed
void delayMillisecondsPedestrian(unsigned long milliseconds) {
  unsigned long startTime = millis();
  while (millis() - startTime < milliseconds) {
    checkPedestrianButton();
  }
}

// Handles the reception of an OFF message by turning off traffic lights and enable the yellow blinking
void receiveOFF() {
  digitalWrite(PL_AG, LOW);
  digitalWrite(PL_AR, LOW);
  digitalWrite(TL_AG, LOW);
  digitalWrite(TL_AR, LOW);
  digitalWrite(TL_BG, LOW);
  digitalWrite(TL_BR, LOW);
  blinkYellow = true;
}

// Handles the yellow bliking
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

// Sets the ack message
void setACK() {
  ack[0] = (char)getEntryNumber();
  ack[1] = (char)ACK;
  ack[2] = (char)controller;
  ack[3] = (char)(getEntryNumber() + ACK + controller);
}

// Sets the status message
void setSTATUS() {
  pedestRedFailing = digitalRead(PL_AR); // Checks the most critical (Pedest Red)
  redFailing = digitalRead(TL_AR); // Checks the most critical (Red)
  char data[] = { pedestRedFailing + '0', pedestYellowFailing + '0', pedestGreenFailing + '0', redFailing + '0', yellowFailing + '0', greenFailing + '0', timerActivated + '0', '0' };
  int information = strtol(data, NULL, 2);
  status[0] = (char)getEntryNumber();
  status[1] = (char)STATUS;
  status[2] = (char)controller;
  status[3] = (char)information;
  status[4] = (char)(getEntryNumber() + information + STATUS + controller);
}

// Triggered when the Access gets interrupted to send a message to the Controller
void requestEvent(int howMany) {
  if (msgType == RED || msgType == GREEN || msgType == OFF) {
    setACK();
    for (int i = 0; i < ACK; i++)
      Wire.write(ack[i]);
  }
  if (msgType == PING) {
    setSTATUS();
    for (int i = 0; i < STATUS; i++)
      Wire.write(status[i]);
  }
}