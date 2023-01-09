#include "Wire.h"

// TODO:  Each traffic control light (green, yellow, and red) must have fault detection capability to detect that it is always turned OFF and does not react to its control
// TODO: It must be possible for the controller and any traffic lights, to detect faults of the communications link, or in other traffic lights
// TODO: While receiving or sending data the controller’s blue LED must blink

const int controller = 0;
const int RED = 0;
const int GREEN = 1;
const int OFF = 2;
const int PING = 3;
const int ACK = 4;
const int STATUS = 5;


// The controller must be designed as an Arduino UNO with I/O devices attached.
// - 2 LEDs: a red LED to show the controller status (ON or OFF), and a blue LED to indicate activity on the communications bus
#define CONTROLLER_STATUS 6
#define BUS_ACTIVITY 5

// - an ON/OFF button (turn on/turn off)
#define BUTTON 7

// - a potentiometer to select the period of traffic control (enter street #1 → enter street #2 → enter street #3 → enter street #4 → enter street #1)
const int potentiometer = A0;

#define JUNCTION_ADDRESS_1 1
#define JUNCTION_ADDRESS_2 2
#define JUNCTION_ADDRESS_3 3
#define JUNCTION_ADDRESS_4 4

#define NUMBER_OF_JUNCTIONS 2 /*4*/

int junctionsArray[NUMBER_OF_JUNCTIONS] = { JUNCTION_ADDRESS_1 , JUNCTION_ADDRESS_2 /*, JUNCTION_ADDRESS_3, JUNCTION_ADDRESS_4*/ };



int lastButtonState;
int currentButtonState;

// Will store last time the second interval occured:
unsigned long previousMillis = 0;
unsigned long remainingTime = 0;
unsigned long reduction = 0;
bool timeIsHalved = 0;

int firstTime = 1;


void setup() {
  Serial.begin(9600);

  pinMode(CONTROLLER_STATUS, OUTPUT);
  pinMode(BUS_ACTIVITY, OUTPUT);

  pinMode(BUTTON, INPUT);

  Wire.begin(0);

  //for (int i = 0; i < NUMBER_OF_JUNCTIONS; i++)
  //Wire.begin(junctionsArray[i]);  // join i2c bus with SLAVE_ADDRESS
}

int controllerON = 0;
int firstSequence = 1;

int junctionIndexToGREEN = 1;
int junctionIndexToRED = 0;
int ft = 1;
int controlPeriodOn = 0;

void loop() {

  if(firstTime) initialState();

  Serial.println("Passei pelo loop()");

  handleButtonPress();

  if(controllerON) {
    // controller turned ON (red LED ON)
    digitalWrite(CONTROLLER_STATUS, HIGH);

    if(firstSequence) initialSequence();
 
    if (!controlPeriodOn) {
      previousMillis = millis();
      remainingTime = getControlPeriod();
      controlPeriodOn = 1;
    }

    Serial.print("1 Millis antes: ");
    Serial.println(millis());

    
    if (remainingTime > 0) {

      //Serial.print("Remaining time: ");
      //Serial.println(remainingTime);
      handleButtonPress();

      handlePedestrianHoldTimePing(junctionIndexToRED);

      if (timeIsHalved && ft) {
        Serial.println("NOOOOOOOOOO");
        remainingTime = remainingTime / 2;
        timeIsHalved = 0;  // DO NOT PREVENT WHEN PEASANT PRESS MORE THAN ONE TIME
        ft = 0;
      }
      reduction = millis() - previousMillis;
      previousMillis = millis();
      if(remainingTime <= reduction)
        remainingTime = 0;
      else
        remainingTime = remainingTime - reduction;
    } else {
      controlPeriodOn = 0;
      ft = 1;

      Serial.print("1 Millis depois: ");
      Serial.println(millis());

      if(junctionIndexToRED == NUMBER_OF_JUNCTIONS-1)
        junctionIndexToGREEN = 0;
      

      char red[] = { (char)controller, (char)RED, (char)junctionsArray[junctionIndexToRED], (char)junctionsArray[junctionIndexToRED] };
      sendMessage(red);

      char green[] = { (char)controller, (char)GREEN, (char)junctionsArray[junctionIndexToGREEN], (char)(1 + junctionsArray[junctionIndexToGREEN]) };
      sendMessage(green);

      if(junctionIndexToRED == NUMBER_OF_JUNCTIONS-1)
        junctionIndexToRED = 0;
      else
        junctionIndexToRED++;
      
      junctionIndexToGREEN++;
    }
                
      
  } else {
    firstSequence = 1;
    // controller turned ON (red LED ON)
    digitalWrite(CONTROLLER_STATUS, LOW);
  }



  
  
}

void handleButtonPress() {
  lastButtonState = currentButtonState;      // Stores the previous state of the push button
  currentButtonState = digitalRead(BUTTON);  // Stores the present state of the push button
  // Detects if the button was pressed again and exists the loop:
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    // When turned OFF, the controller must signal all traffic lights to start blinking yellow, going back to the initial state
    if (controllerON) {
      Serial.println("DESLIGAAAAAAAAAR");
      firstTime = 1; 
      controllerON = 0;
     // break;     
    } else {
      Serial.println("LIGAAAAAAAAAAR");
      controllerON = 1;
    }
    Serial.println("VERMELHÃO PRESSIONADOOOOO");
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

// Initial state of the system must
void initialState() {

  firstTime = 0;
  // controller turned OFF (red LED OFF)
  digitalWrite(CONTROLLER_STATUS, LOW);

  Serial.println("INITIAL STATEEEEEEEEEEEEEEE");

  // all traffic lights blinking yellow with a 1 second period (ON + OFF cycle time)
  for (int junctionIndex = 0; junctionIndex < NUMBER_OF_JUNCTIONS; junctionIndex++) {
    char message[] = { (char)controller, (char)OFF, (char)junctionsArray[junctionIndex], (char)(OFF + junctionsArray[junctionIndex]) };
    sendMessage(message);
  }

  
}

// Wait a control period [2, 15] seconds (controlled by potentiometer).
int getControlPeriod() {
  int time = map(analogRead(potentiometer), 0, 1023, 2000, 15000);
  return time;
}

// When the controller wants to send a message he should do:
void sendMessage(char message[]) {
  int destination = (int)message[2];
  int msgType = (int)message[1];

  Serial.print("ENVIAR ");
  Serial.print(msgType);
  Serial.print(" para ");
  Serial.println(destination);

  // While receiving or sending data the controller’s blue LED must blink.
  startBlinkingBLUE();

  Wire.beginTransmission(destination);  // transmit to device
  Wire.write(message[0]);                  // sends the message
  Wire.write(message[1]); 
  Wire.write(message[2]); 
  Wire.write(message[3]); 
  Wire.endTransmission();

  if (msgType == PING) {
    Serial.println("VOU PEDIR UM STATUSSSSSSSSSSSSS");
    
    Wire.requestFrom(destination, STATUS);  // request 5 bytes from slave device when a PING was sent
    
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

    checkStatus((int)information);
    Serial.print("Received STATUS : ");
    Serial.print((int)information);  // DO SOMETHING WITH THE ACKs ?????????????????
    Serial.print(" from: ");
    Serial.println((int)sender);
    //Serial.println(c);  // DO SOMETHING WITH THE ACKs ?????????????????
    
  } else {
    Wire.requestFrom(destination, ACK);  // request 4 bytes from slave device when another message was sent
    // DO SOMETHING WHEN HE RECEIVES A TIME(X) MESSAGE
    char sender = Wire.read();
    char type = Wire.read();
    char destination = Wire.read();
    char sender_ack = Wire.read();
    Serial.print("Received ACK : ");
    Serial.print((int)sender_ack);  // DO SOMETHING WITH THE ACKs ?????????????????
    Serial.print(" from: ");
    Serial.println((int)sender);

  }

  // While receiving or sending data the controller’s blue LED must blink.
  stopBlinkingBLUE();
}

// While receiving or sending data the controller’s blue LED must blink.
void startBlinkingBLUE() {
  digitalWrite(BUS_ACTIVITY, HIGH); // TODO
}

// While receiving or sending data the controller’s blue LED must blink.
void stopBlinkingBLUE() {
  digitalWrite(BUS_ACTIVITY, LOW); // TODO
}

void checkStatus(int status) {
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