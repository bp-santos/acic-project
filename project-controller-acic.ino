#include "Wire.h"

// TODO:  Each traffic control light (green, yellow, and red) must have fault detection capability to detect that it is always turned OFF and does not react to its control
// TODO: It must be possible for the controller and any traffic lights, to detect faults of the communications link, or in other traffic lights

// Controller identification
const int controller = 0;

// Types of messages
const int RED = 0;
const int GREEN = 1;
const int OFF = 2;
const int PING = 3;
const int ACK = 4;
const int STATUS = 5;

// Controller pins
#define CONTROLLER_STATUS 12
//#define BUS_ACTIVITY 5
#define CONTROLLER_BUTTON 13
const int potentiometer = A0;

// Junctions
#define JUNCTION_ADDRESS_1 1
#define JUNCTION_ADDRESS_2 2
#define JUNCTION_ADDRESS_3 3
#define JUNCTION_ADDRESS_4 4
#define NUMBER_OF_JUNCTIONS 2 /*4*/
int junctionsArray[NUMBER_OF_JUNCTIONS] = { JUNCTION_ADDRESS_1 , JUNCTION_ADDRESS_2 /*, JUNCTION_ADDRESS_3, JUNCTION_ADDRESS_4*/ };

// Variables to handle Controller Button
int lastControllerButtonState;
int currentControllerButtonState;

// Variables to handle Control Period
int pedestrianPressedOnce = 0;
int controlPeriodOn = 0;
unsigned long previousMillis = 0;
unsigned long remainingTime = 0;
unsigned long reduction = 0;
bool timeIsHalved = 0;
#define MIN_CONTROL_PERIOD 2000
#define MAX_CONTROL_PERIOD 15000

// Flag to make the initial state perform only once at the start of the system
int firstTimeInLoop = 1;

// Flag to set the system ON and OFF
int controllerON = 0;

// Flag to make the system perform the initial sequence (block entries 2, 3 and 4 and command entry 1 to GREEN) 
// perform only once at the beggining
int firstSequence = 1;

// Variables to handle the next junction sequence
int junctionIndexToRED = 0; // Index of the junction wich entry goes RED
int junctionIndexToGREEN = 1; // Index of the junction wich entry goes GREEN


// ------------ FROM ACCESS CODE BUT ADAPTED: -------------------
// Jumpers pins
const byte jumper1 = 3;
const byte jumper0 = 2; 

// Traffic lights
#define NUMBER_OF_TRAFFIC_LIGHTS 7
#define TL_AR 7
#define TL_AY 8
#define TL_AG 9
#define PL_AG 10
#define PEDESTRIAN_BUTTON 11
#define TL_BR 4
#define TL_BY 5
#define TL_BG 6
int lights[NUMBER_OF_TRAFFIC_LIGHTS] = { TL_BR, TL_BY, TL_BG, TL_AR, TL_AY, TL_AG, PL_AG };

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

int myEntryNumber;

// Setup
void setup() {
  Serial.begin(9600);

  // ------------ FROM ACCESS CODE BUT ADAPTED: -------------------
  // Initializes the jumpers as inputs
  pinMode(jumper1, INPUT);
  pinMode(jumper0, INPUT);
  // join i2c bus with address #entryNumber
  myEntryNumber = getEntryNumber();
  Serial.print("ENTRY ");
  Serial.println(myEntryNumber);

  // Initializes the traffic lights as outputs
  for (int i = 0; i < NUMBER_OF_TRAFFIC_LIGHTS; i++)
    pinMode(lights[i], OUTPUT);
  
  // Initializes the pedestrian button as an input
  pinMode(PEDESTRIAN_BUTTON, INPUT);

  // -------------- Controller: -------------

  pinMode(CONTROLLER_STATUS, OUTPUT);
  //pinMode(BUS_ACTIVITY, OUTPUT);

  pinMode(CONTROLLER_BUTTON, INPUT);

  Wire.begin(controller);
}

// Loop
void loop() {
  // ------------ FROM ACCESS CODE BUT ADAPTED: -------------------
  checkPedestrianButton();
  if (blinkYellow) handleYellowBlink();

  // ------------ Controller: -------------

  if(!controllerON && firstTimeInLoop) initialState();
  handleControllerButtonPress();

  if(controllerON) {
    if(firstSequence) initialSequence();
 
    if (!controlPeriodOn) setsControlPeriodOnVariables();

    if (remainingTime > 0) {
      controlPeriod();
    } else {
      setsControlPeriodOffVariables();
      handlesNextJunctionInSequence();
    }              
  } else {
    firstSequence = 1;
  }

}

void controlPeriod() {
  handleControllerButtonPress();
  handlePedestrianHoldTimePing(junctionIndexToRED); // TALVEZ AGR POSSA SER FORA? VER!!!
  if (timeIsHalved && !pedestrianPressedOnce) {
    Serial.println("NOOOOOOOOOO");
    remainingTime = remainingTime / 2;
    timeIsHalved = 0;
    pedestrianPressedOnce = 1;
  }
  reduction = millis() - previousMillis;
  previousMillis = millis();
  if(remainingTime <= reduction)
    remainingTime = 0;
  else
    remainingTime = remainingTime - reduction;
}

// Sets all needed variables to when the control period gets ON
void setsControlPeriodOnVariables() {
  previousMillis = millis(); // Gets the present time
  remainingTime = getControlPeriod(); // Gets the remaining time of the control period
  controlPeriodOn = 1; // Sets the control period ON
  pedestrianPressedOnce = 0;
}

void setsControlPeriodOffVariables() {
  controlPeriodOn = 0; // a new juntion is going to be analysed
  pedestrianPressedOnce = 0; // a new juntion is going to be analysed
  timeIsHalved = 0;
}

void handlesNextJunctionInSequence() {
  if(junctionIndexToRED == NUMBER_OF_JUNCTIONS-1) // Reaches the limit of junctions to go red
    junctionIndexToGREEN = 0; // Sets junction that is going green as the first junction
  
  char red[] = { (char)controller, (char)RED, (char)junctionsArray[junctionIndexToRED], (char)junctionsArray[junctionIndexToRED] };
  sendMessage(red);

  char green[] = { (char)controller, (char)GREEN, (char)junctionsArray[junctionIndexToGREEN], (char)(1 + junctionsArray[junctionIndexToGREEN]) };
  sendMessage(green);

  if(junctionIndexToRED == NUMBER_OF_JUNCTIONS-1) // Reaches the limit of junctions to go red
    junctionIndexToRED = 0; // Next juntion to go red is the first one
  else
    junctionIndexToRED++;
  
  junctionIndexToGREEN++;
}

void handleControllerButtonPress() {
  lastControllerButtonState = currentControllerButtonState;      // Stores the previous state of the push button
  currentControllerButtonState = digitalRead(CONTROLLER_BUTTON);  // Stores the present state of the push button
  // Detects if the button was pressed again and exists the loop:
  if (lastControllerButtonState == HIGH && currentControllerButtonState == LOW) {
    // When turned OFF, the controller must signal all traffic lights to start blinking yellow, going back to the initial state
    if (controllerON) {
      Serial.println("DESLIGAR");
      firstTimeInLoop = 1; 
      controllerON = 0;
      digitalWrite(CONTROLLER_STATUS, LOW); // controller turned OFF (red LED ON)    
    } else {
      Serial.println("LIGAR");
      controllerON = 1;
      digitalWrite(CONTROLLER_STATUS, HIGH); // controller turned ON (red LED ON)
    }
  }
}

int pedestrianButtonCheck = 1000;
unsigned long previousMillisBlink = 0;
void handlePedestrianHoldTimePing(int junctionIndexToRED) {
  if (millis() - previousMillisBlink > pedestrianButtonCheck) {
    char ping[] = {(char)controller, (char)PING, (char)junctionsArray[junctionIndexToRED], (char)(PING + junctionsArray[junctionIndexToRED])};
    sendMessage(ping);
    previousMillisBlink = millis();
  }
}

void initialSequence() {
  firstSequence = 0;
  // Command entries 2, 3 and 4 to go RED, wait for the acknowledgements, command entry 1 to go GREEN
  for (int i = 1; i < NUMBER_OF_JUNCTIONS; i++) {
    char red[] = { (char)controller, (char)RED, (char)junctionsArray[i], (char)junctionsArray[i] };
    sendMessage(red);
  }
  char green[] = { (char)controller, (char)GREEN, (char)junctionsArray[0], (char)(1 + junctionsArray[0]) };
  sendMessage(green);
}

// Performs initial state of the system
void initialState() {
  firstTimeInLoop = 0;
  // controller turned OFF (red LED OFF)
  digitalWrite(CONTROLLER_STATUS, LOW);

  Serial.println("INITIAL STATE");

  // all traffic lights blinking yellow with a 1 second period (ON + OFF cycle time)
  for (int junctionIndex = 0; junctionIndex < NUMBER_OF_JUNCTIONS; junctionIndex++) {
    char message[] = { (char)controller, (char)OFF, (char)junctionsArray[junctionIndex], (char)(OFF + junctionsArray[junctionIndex]) };
    sendMessage(message);
  }
}

// Gets control period
int getControlPeriod() {
  return map(analogRead(potentiometer), 0, 1023, MIN_CONTROL_PERIOD, MAX_CONTROL_PERIOD);
}

// Sends message to controller
void sendMessage(char message[]) {
  int destination = (int)message[2];
  int msgType = (int)message[1];

  Serial.print("ENVIAR ");
  Serial.print(msgType);
  Serial.print(" para ");
  Serial.println(destination);


  if (destination == myEntryNumber) {
    switch(msgType) {
      case RED: receiveRED(); break;
      case GREEN: receiveGREEN(); break;
      case OFF: receiveOFF(); break;
      case PING: break;
      default: Serial.println("Should never happen, but just in case...");
    }
  } else {

    startBlinkingBLUE();

    Wire.beginTransmission(destination);  // transmit to device
    Wire.write(message[0]);                  
    Wire.write(message[1]); 
    Wire.write(message[2]); 
    Wire.write(message[3]); 
    Wire.endTransmission();

    stopBlinkingBLUE();

    if (msgType == PING) {
      Serial.println("STATUS");
      
      Wire.requestFrom(destination, STATUS);  // request 5 bytes from slave device when a PING was sent
      
      startBlinkingBLUE();

      char sender = Wire.read();
      Serial.print("Sender: ");
      Serial.println((int)sender);
      char type = Wire.read();
      Serial.print("Type: ");
      Serial.println((int)type);
      char destination = Wire.read();
      Serial.print("Destination: ");
      Serial.println((int)destination);
      char information = Wire.read();
      Serial.print("Information: ");
      Serial.println((int)information);
      char sender_status = Wire.read();
      Serial.print("Sender status: ");
      Serial.println((int)sender_status);

      stopBlinkingBLUE();

      checkStatus((int)information);
      
      // DO SOMETHING WITH THE ACKs ?????????????????
      
    } else {
      Wire.requestFrom(destination, ACK);  // request 4 bytes from slave device when another message was sent
      // DO SOMETHING WHEN HE RECEIVES A TIME(X) MESSAGE
      startBlinkingBLUE();
      char sender = Wire.read();
      char type = Wire.read();
      char destination = Wire.read();
      char sender_ack = Wire.read();
      stopBlinkingBLUE();
      Serial.print("Received ACK : ");
      Serial.print((int)sender_ack);  // DO SOMETHING WITH THE ACKs ?????????????????
      Serial.print(" from: ");
      Serial.println((int)sender);

    }

  }
}

// While receiving or sending data the controller’s blue LED must blink.
void startBlinkingBLUE() {
  digitalWrite(CONTROLLER_STATUS, HIGH); // CONTROLLER_STATUS substitutes BUS_ACTIVITY
}

// While receiving or sending data the controller’s blue LED must blink.
void stopBlinkingBLUE() {
  digitalWrite(CONTROLLER_STATUS, LOW); // CONTROLLER_STATUS substitutes BUS_ACTIVITY
}

void checkStatus(int status) { // TODO - completar o check do estado
  Serial.println("Checking status...");
  int binary[8];
  for (int i = 0; i < 8; i++)
    binary[i] = 0;
  int i = 7;
  while (status > 0) {
    binary[i] = status % 2;
    status = status / 2;
    i--;
  }
  // DO SOMETHING WITH THE FAULTY LIGHTS ????????????????

  if (binary[6] == 1)
    timeIsHalved = 1;
}



// ------------ FROM ACCESS CODE BUT ADAPTED: -------------------


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

  int number = strtol(entry, NULL, 2);
  return number + 1;
}

// Checks if the pedestrian button was pressed
void checkPedestrianButton() {
  lastPedestrianButtonState = currentPedestrianButtonState;      // Stores the previous state of the push button
  currentPedestrianButtonState = digitalRead(PEDESTRIAN_BUTTON);  // Stores the present state of the push button
  if (lastPedestrianButtonState == HIGH && currentPedestrianButtonState == LOW) {
    timerActivated = 1;
    timeIsHalved = 1;
    Serial.println("FUI CARREGADO!!!!!!!!");
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
  digitalWrite(PL_AG, HIGH); // Pedestrian light turns GREEN
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
  digitalWrite(PL_AG, LOW); // Pedestrian light turns RED
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
  //digitalWrite(PL_AR, LOW);
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