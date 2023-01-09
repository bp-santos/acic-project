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
#define CONTROLLER_STATUS 6
#define BUS_ACTIVITY 5
#define CONTROLLER_BUTTON 7
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


// Setup
void setup() {
  Serial.begin(9600);

  pinMode(CONTROLLER_STATUS, OUTPUT);
  pinMode(BUS_ACTIVITY, OUTPUT);

  pinMode(CONTROLLER_BUTTON, INPUT);

  Wire.begin(0);
}

// Loop
void loop() {
  Serial.println("Passei pelo loop()");
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
    timeIsHalved = 0;  // DO NOT PREVENT WHEN PEASANT PRESS MORE THAN ONE TIME
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
}

void setsControlPeriodOffVariables() {
  controlPeriodOn = 0; // a new juntion is going to be analysed
  pedestrianPressedOnce = 0; // a new juntion is going to be analysed
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
  return map(analogRead(potentiometer), 0, 1023, 2000, 15000);
}

// Sends message to controller
void sendMessage(char message[]) {
  int destination = (int)message[2];
  int msgType = (int)message[1];

  Serial.print("ENVIAR ");
  Serial.print(msgType);
  Serial.print(" para ");
  Serial.println(destination);

  startBlinkingBLUE();

  Wire.beginTransmission(destination);  // transmit to device
  Wire.write(message[0]);                  // sends the message
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

// While receiving or sending data the controller’s blue LED must blink.
void startBlinkingBLUE() {
  digitalWrite(BUS_ACTIVITY, HIGH);
}

// While receiving or sending data the controller’s blue LED must blink.
void stopBlinkingBLUE() {
  digitalWrite(BUS_ACTIVITY, LOW);
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