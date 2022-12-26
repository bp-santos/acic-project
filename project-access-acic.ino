#include "Wire.h"
#include <stdlib.h>

const int controller = 0;
#define NUMBER_OF_LIGHTS = 8;

// Each access – k – has two traffic lights (TL kA, TL kB).
// TL kA controls:
// - accesses of vehicles to enter the roundabout
#define TL_AG 1
#define TL_AY 2
#define TL_AR 3

// - also has lights and a press button for pedestrians to signal their intent to cross the street (PL kA)
// To simplify the assembly of the circuit only one set of pedestrian lights and button will be implemented on one side of the 
street.
#define BUTTON 4
#define PL_AG 5
#define PL_AR 6

// TL kB controls the flow of vehicles in the roundabout
// - prevents the passage of vehicles once the traffic in the nearby street is allowed in the roundabout
// - only vehicles turning right, to get off the roundabout, are allowed to proceed
// The coordination between TL kA and TL kB must be self-sufficient, implemented locally and not by the roundabout controller
#define TL_BG 7
#define TL_BY 8
#define TL_BR 9


byte lights[] = {TL_AG, TL_AY, TL_AR, PL_AG, PL_AR, TL_BG, TL_BY, TL_BR};

const long interval = 500;

const bool keepBlinking = false;

int pedestRedFailing = 0;
int pedestYellowFailing = 0;
int pedestGreenFailing = 0;
int redFailing = 0;
int yellowFailing = 0;
int greenFailing = 0;
int timerActivated = 0;

int information;

byte ack[4];
byte status[5];

void setup(){
    Serial.begin(9600); // Starts serial for output
    int entryNumber = getEntryNumber();
    Wire.begin(entryNumber); // join i2c bus with address #entryNumber
    Wire.onReceive(receiveEvent); // when master sends a message
    Wire.onRequest(requestEvent); //when master asks for the answer
    pinMode(BUTTON, INPUT); // Initializes the pedestrian button as an input
    for(int i = 0; i < NUMBER_OF_LIGHTS; i++)
        pinMode(lights[i], OUTPUT); // Initializes the traffic lights as outputs
}

void getEntryNumber(){

}

// Triggered when the Access gets interrupted to read the message from the Controller
void receiveEvent(int i) {
    if ( i == 4 || i == 5){ // Extra security check (how many bytes were sent)
        char sender = Wire.read();
        char operationNumber = Wire.read();
        char destination = Wire.read();
        char integrityByte;                     // WHAT TO DO WITH THIS ??????
        char information;
        if (i == 4)
            integrityByte = Wire.read();
        if (i == 5){
            information = Wire.read();
            integrityByte = Wire.read();
        }
        if(destination == getEntryNumber()){
            switch(sender) {
                case controller:
                    switch(operationNumber){
                        case '0': receiveRED(); break;
                        case '1': receiveGREEN(); break;
                        case '2': receiveOFF(); break;
                        case '3': receivePING(); break;
                        //case '4': receiveACK(); break;
                        //case '5': receiveSTATUS(); break;
                        default: Serial.println("Should never happen, but just in case...");
                    }
                    break;
                default: Serial.println("Should never happen, but just in case...");
            }
        }
    }
}

// The coordination between TL kA and TL kB must be self-sufficient, implemented locally and not by the roundabout controller
void receiveRED(){
    // Stop blinking YELLOW (turned ON when OFF was received)
    keepBlinking = false;

    // Access light turns YELLOW
    digitalWrite(TL_AG, LOW);
    digitalWrite(TL_AY, HIGH);

    // Each passage through Yellow will take 0,5 seconds
    delay(interval);

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
    delay(interval);

    // Roundabout light turns GREEN
    digitalWrite(TL_BY, LOW);
    digitalWrite(TL_BG, HIGH);

    setACK();
}

// The coordination between TL kA and TL kB must be self-sufficient, implemented locally and not by the roundabout controller
void receiveGREEN(){
    // Stop blinking YELLOW (turned ON when OFF was received)
    keepBlinking = false;

    // Pedestrian light turns RED
    digitalWrite(PL_AG, LOW);
    digitalWrite(PL_AR, HIGH);

    // Roundabout light turns YELLOW
    digitalWrite(TL_BG, LOW);
    digitalWrite(TL_BY, HIGH);

    // Each passage through Yellow will take 0,5 seconds
    delay(interval);

    // Roundabout light turns RED
    digitalWrite(TL_BY, LOW);
    digitalWrite(TL_BR, HIGH);

    // Access light turns YELLOW
    digitalWrite(TL_AR, LOW);
    digitalWrite(TL_AY, HIGH);

    // Each passage through Yellow will take 0,5 seconds
    delay(interval);

    // Access light turns GREEN
    digitalWrite(TL_AY, LOW);
    digitalWrite(TL_AG, HIGH);

    setACK();
}

void receiveOFF(){
    digitalWrite(PL_AG, LOW);
    digitalWrite(PL_AR, LOW);
    digitalWrite(TL_AG, LOW);
    digitalWrite(TL_AR, LOW);
    digitalWrite(TL_BG, LOW);
    digitalWrite(TL_BR, LOW);

    setACK();

    keepBlinking = true;

    // Start blinking YELLOW (will be turned OFF when RED/GREEN is received)
    while (keepBlinking){
        digitalWrite(TL_AY, HIGH);
        digitalWrite(TL_BY, HIGH);

        // Each passage through Yellow will take 0,5 seconds
        delay(interval);

        digitalWrite(TL_AY, LOW);
        digitalWrite(TL_BY, LOW);

        // Each passage through Yellow will take 0,5 seconds
        delay(interval);
    }
}

// The traffic lights will never do Ping(x) to other traffic lights neither to controller.
void receivePING(){
    char data[] = {pedestRedFailing + '0', pedestYellowFailing + '0', pedestGreenFailing + '0', redFailing + '0', yellowFailing + '0', greenFailing + '0', timerActivated + '0', '0'};
    information = strtol(data, NULL, 2);
    setSTATUS();
}

// The ACK (x) should be send as response to RED(x), GREEN(x) and OFF(x) requests.
void setACK(){
    ack = {getEntryNumber(), 4, controller, getEntryNumber() + 4 + controller};
}

// Status(X) will be the response from the traffic light when the controller do a Ping(x) request.
// The value which was defined to be send in TIME(x) now is obtained by the controller in the Status(X) message.               ???????????
void setSTATUS(int information){
    status = {getEntryNumber(), 5, controller, information, getEntryNumber() + information + 5 + controller};
}

void requestEvent(int i){
    if (i == 4) // Extra security check (how many bytes were sent)
        Wire.write(ack);
    if (i == 5) // Extra security check (how many bytes were sent)
        Wire.write(status);
}

//  The value which was defined to be send in TIME(x) now is obtained by the controller in the Status(X) message.               ???????????
void sendTIME(){

}

void loop(){
    lastButtonState = currentButtonState;   // Stores the previous state of the push button
    currentButtonState = digitalRead(BUTTON);   // Stores the present state of the push button

    if (lastButtonState == HIGH && currentButtonState == LOW)
        // Loops while the button is not pressed again:
        sendTIME();
}