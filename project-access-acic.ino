#include "Wire.h"
#include <stdlib.h>

const int controller = 0;
#define NUMBER_OF_LIGHTS 8

const int RED = 0;
const int GREEN = 1;
const int OFF = 2;
const int PING = 3;
const int ACK = 4;
const int STATUS = 5;

bool msgReceived = false;
int msgType = -1;

// Each access – k – has two traffic lights (TL kA, TL kB).
// The identification of the roundabout entry corresponding to the traffic light is configured by jumpers (or fixed wires) connected to input ports of the Arduino controller – 1, 2, 3, 4 – in ascending order anti-clockwise.
const byte jumper2 = 4;
const byte jumper1 = 3;
const byte jumper0 = 2; 

// TL kA controls:
// - accesses of vehicles to enter the roundabout
#define TL_AR 10
#define TL_AY 9
#define TL_AG 8

// - also has lights and a press button for pedestrians to signal their intent to cross the street (PL kA)
// To simplify the assembly of the circuit only one set of pedestrian lights and button will be implemented on one side of the street.
#define PL_AR 7
#define PL_AG 6
#define BUTTON 5

// TL kB controls the flow of vehicles in the roundabout
// - prevents the passage of vehicles once the traffic in the nearby street is allowed in the roundabout
// - only vehicles turning right, to get off the roundabout, are allowed to proceed
// The coordination between TL kA and TL kB must be self-sufficient, implemented locally and not by the roundabout controller
#define TL_BR 13
#define TL_BY 12
#define TL_BG 11


int lights[NUMBER_OF_LIGHTS] = { TL_BR, TL_BY, TL_BG, TL_AR, TL_AY, TL_AG, PL_AR, PL_AG };

const int yellowInterval = 500;

bool blinkYellow = false;

int pedestRedFailing = 0;
int pedestYellowFailing = 0;
int pedestGreenFailing = 0;
int redFailing = 0;
int yellowFailing = 0;
int greenFailing = 0;
int timerActivated = 0;

int lastButtonState;
int currentButtonState;

char ack[4];
char status[5];

void setup() {
  // Starts serial for output
  Serial.begin(9600);


  // Initializes the pedestrian button as an input
  pinMode(BUTTON, INPUT);

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
  for (int i = 0; i < NUMBER_OF_LIGHTS; i++)
    pinMode(lights[i], OUTPUT);
}

void loop() {
  lastButtonState = currentButtonState;      // Stores the previous state of the push button
  currentButtonState = digitalRead(BUTTON);  // Stores the present state of the push button

  if (lastButtonState == HIGH && currentButtonState == LOW)
    // Loops while the button is not pressed again:
    sendTIME();
  
  if (blinkYellow) handleYellowBlink();

  if(msgReceived) {
    switch (msgType) {
      case RED: receiveRED(); msgReceived = false; break;
      case GREEN: receiveGREEN(); msgReceived = false; break;
      case OFF: receiveOFF(); msgReceived = false; break;
      case PING: receivePING(); msgReceived = false; break;
      //case 4: receiveACK(); break;
      //case 5: receiveSTATUS(); break;
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

// Triggered when the Access gets interrupted to read the message from the Controller
void receiveEvent(int howMany) {

  if (howMany == 4 || howMany == 5) {  // Extra security check (how many bytes were sent)
    char sender = Wire.read();
    char operationNumber = Wire.read();
    char destination = Wire.read();
    char integrityByte;  // WHAT TO DO WITH THIS ??????
    char information;
    if (howMany == 4)
      integrityByte = Wire.read(); // TODO: CHECK IF MESSAGE IS VALID ACCORDINGLY TO THE PROTOCOL
    if (howMany == 5) {
      information = Wire.read(); // TODO: DISCOVER WHY USE THIS SHIT
      integrityByte = Wire.read();
    }
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
            //case 4: receiveACK(); break;
            //case 5: receiveSTATUS(); break;
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

  // Roundabout light turns YELLOW
  digitalWrite(TL_BR, LOW);
  digitalWrite(TL_BY, HIGH);

  // Each passage through Yellow will take 0,5 seconds
  delayMilliseconds(yellowInterval);

  // Roundabout light turns GREEN
  digitalWrite(TL_BY, LOW);
  digitalWrite(TL_BG, HIGH);

  setACK();

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
  delayMilliseconds(yellowInterval);

  

  // Roundabout light turns RED
  digitalWrite(TL_BY, LOW);
  digitalWrite(TL_BR, HIGH);

  // Access light turns YELLOW
  digitalWrite(TL_AR, LOW);
  digitalWrite(TL_AY, HIGH);

  

  // Each passage through Yellow will take 0,5 seconds
  delayMilliseconds(yellowInterval);

  

  // Access light turns GREEN
  digitalWrite(TL_AY, LOW);
  digitalWrite(TL_AG, HIGH);

  setACK();

  Serial.print("GREEN FIM: ");
  Serial.println(millis());
}

void delayMilliseconds(unsigned long milliseconds) {
  unsigned long startTime = millis();
  while (millis() - startTime < milliseconds) {
    // Do nothing
  }
}


unsigned long previousMillisBlink = 0; // Will store last time LED was updated
int blinkInterval = 1000; 
int ledState = LOW;
void receiveOFF() {
  digitalWrite(PL_AG, LOW);
  digitalWrite(PL_AR, LOW);
  digitalWrite(TL_AG, LOW);
  digitalWrite(TL_AR, LOW);
  digitalWrite(TL_BG, LOW);
  digitalWrite(TL_BR, LOW);

  setACK();

  Serial.println("OFF RECEIVED!!!!!!");

  blinkYellow = true;
}

void handleYellowBlink() {
  if (millis() - previousMillisBlink > blinkInterval) {
    // If the LED is OFF turn it ON and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    // Sets the LEDs with the ledState (HIGH/LOW):
    digitalWrite(TL_AY, ledState);
    digitalWrite(TL_BY, ledState);
    previousMillisBlink = millis();
  }
}

// The traffic lights will never do Ping(x) to other traffic lights neither to controller.
void receivePING() {
  char data[] = { pedestRedFailing + '0', pedestYellowFailing + '0', pedestGreenFailing + '0', redFailing + '0', yellowFailing + '0', greenFailing + '0', timerActivated + '0', '0' };
  int information = strtol(data, NULL, 2);
  setSTATUS(information); 
  timerActivated = 0;
}

// The ACK (x) should be send as response to RED(x), GREEN(x) and OFF(x) requests.
void setACK() {
  ack[0] = (char)getEntryNumber();
  ack[1] = (char)ACK;
  ack[2] = (char)controller;
  ack[3] = (char)(getEntryNumber() + 4 + controller);
  //{ (char)getEntryNumber(), (char)4, (char)controller, (char)(getEntryNumber() + 4 + controller) };
}

// Status(X) will be the response from the traffic light when the controller do a Ping(x) request.
void setSTATUS(int information) {
  status[0] = (char)getEntryNumber();
  status[1] = (char)5;
  status[2] = (char)controller;
  status[3] = (char)information;
  status[4] = (char)(getEntryNumber() + information + 5 + controller);
  // { (char)getEntryNumber(), (char)5, (char)controller, (char)information, (char)(getEntryNumber() + information + 5 + controller) };
}

void requestEvent(int howMany) {
  if (howMany == 4) {  // Extra security check (how many bytes were sent)
    
    Wire.write(ack[0]);
    Wire.write(ack[1]);
    Wire.write(ack[2]);
    Wire.write(ack[3]);
    
  }
  if (howMany == 5) {  // Extra security check (how many bytes were sent)
    
    Wire.write(status[0]);
    Wire.write(status[1]);
    Wire.write(status[2]);
    Wire.write(status[3]);
    Wire.write(status[4]);
    
  }
}

//  The value which was defined to be send in TIME(x) now is obtained by the controller in the Status(X) message.               ???????????
void sendTIME() {
  timerActivated = 1;
}