#include "Wire.h"

// TODO:  Each traffic control light (green, yellow, and red) must have fault detection capability to detect that it is always turned OFF and does not react to its control
// TODO: It must be possible for the controller and any traffic lights, to detect faults of the communications link, or in other traffic lights
// TODO: While receiving or sending data the controller’s blue LED must blink

const int controller = 0;

// The controller must be designed as an Arduino UNO with I/O devices attached.
// - 2 LEDs: a red LED to show the controller status (ON or OFF), and a blue LED to indicate activity on the communications bus
#define CONTROLLER_STATUS 5
#define BUS_ACTIVITY 6

// - an ON/OFF button (turn on/turn off)
#define BUTTON 7

// - a potentiometer to select the period of traffic control (enter street #1 → enter street #2 → enter street #3 → enter street #4 → enter street #1)
const int potentiometer = A0;

#define JUNCTION_ADDRESS_1 1
#define JUNCTION_ADDRESS_2 2
#define JUNCTION_ADDRESS_3 3
#define JUNCTION_ADDRESS_4 4

#define NUMBER_OF_JUNCTIONS 4

byte junctionsArray[NUMBER_OF_JUNCTIONS] = {JUNCTION_ADDRESS_1, JUNCTION_ADDRESS_2, JUNCTION_ADDRESS_3, JUNCTION_ADDRESS_4};

int lastButtonState;
int currentButtonState;

// Will store last time the second interval occured:
unsigned long previousMillis = 0;  
unsigned long remainingTime = 0;
unsigned long reduction = 0;
bool timeIsHalved = 0;


void setup() { 
    Serial.begin(9600);

    pinMode(CONTROLLER_STATUS, OUTPUT);
    pinMode(BUS_ACTIVITY, OUTPUT);

    pinMode(BUTTON, INPUT);

    for(int i = 0; i < NUMBER_OF_JUNCTIONS; i++)
        Wire.begin(junctionsArray[i]); // join i2c bus with SLAVE_ADDRESS
}

void loop(){
    initialState();
    
    // It must be possible to turn the controller ON and OFF, pressing the button:
    lastButtonState = currentButtonState;   // Stores the previous state of the push button
    currentButtonState = digitalRead(BUTTON);   // Stores the present state of the push button

    if (lastButtonState == HIGH && currentButtonState == LOW) {
        // Loops while the button is not pressed again:
        while(true) {
            lastButtonState = currentButtonState;   // Stores the previous state of the push button
            currentButtonState = digitalRead(BUTTON);   // Stores the present state of the push button

            // Detects if the button was pressed again and exists the loop:
            if (lastButtonState == HIGH && currentButtonState == LOW){
                // When turned OFF, the controller must signal all traffic lights to start blinking yellow, going back to the initial state
                break;
            }

            // When turned ON, the controller must start a cyclic sequence of control of the roundabout. 
            for(int i = 0; i < NUMBER_OF_JUNCTIONS; i++){
                // While one traffic light is performing a red-yellow-green-yellow-red cycle, the other must have its red light always on (and, in part, pedestrian green). 
                //  Block entries 2, 3 and 4 - Command entries 2, 3 and 4 to go RED, wait for the acknowledgements, command entry 1 to go GREEN.
                if (i == 0){
                    sendMessage(char message[] = {controller, 0, junctionsArray[i + 1], junctionsArray[i + 1]});
                    sendMessage(char message[] = {controller, 0, junctionsArray[i + 2], junctionsArray[i + 2]});
                    sendMessage(char message[] = {controller, 0, junctionsArray[i + 3], junctionsArray[i + 3]});
                    sendMessage(char message[] = {controller, 1, junctionsArray[i], 1 + junctionsArray[i]});
                }

                // Wait a control period [2, 15] seconds (controlled by potentiometer).
                // It must be possible to shorten the cycle time by half by pressing the pedestrian button - when the button is pressed the remaining of the cycle time is halved.
                previousMillis = millis();
                //previousMillis = 1000;
                reduction = millis() - previousMillis;
                // reduction = 1000 - 1000 = 0;
                remainingTime = getControlPeriod() - reduction;
                //remainingTime = 1000 - (1000 - 1000) = 1000;
                while(remainingTime > 0){
                    if (timeIsHalved){
                        remainingTime = remainingTime / 2;
                        reduction = millis() - previousMillis;
                        // remainingTime = 425;
                        // reduction = 1200 - 1000 = 200
                        timeIsHalved = 0;                       // DO NOT PREVENT WHEN PEASANT PRESS MORE THAN ONE TIME
                    }
                    remainingTime = remainingTime + reduction;
                    reduction = millis() - previousMillis;
                    remainingTime = remainingTime - reduction;
                    // remainingTime = 1000 + 0  - (1050 - 1000) = 950
                    // remainingTime = 950 + 50  - (1100 - 1000) = 900
                    // remainingTime = 900 + 100 - (1150 - 1000) = 850
                    // PRESS THE BUTTON -> remainingTime = 425; reduction = 1200 - 1000 = 200
                    // remainingTime = 425 + 200 - (1205 - 1000) = 420
                    // remainingTime = 420 + 210 - (1260 - 1000) = 630 - 260 = 370
                    // etc...
                }

                // Block entry 1 – command entry 1 to go RED, wait for the acknowledgement, command entry 2 to go GREEN, and so on
                sendMessage(char message[] = {controller, 0, junctionsArray[i], junctionsArray[i]});
                if (i == 3)
                    sendMessage(char message[] = {controller, 1, junctionsArray[0], 1 + junctionsArray[0]});
                if (i == 0 || i == 1 || i == 2)
                    sendMessage(char message[] = {controller, 1, junctionsArray[i + 1], 1 + junctionsArray[i + 1]});
            }
        }

    }
}

// Initial state of the system must
void initialState(){
    // controller turned OFF (red LED OFF)
    digitalWrite(CONTROLLER_STATUS, LOW);

    // all traffic lights blinking yellow with a 1 second period (ON + OFF cycle time)
    for(int i = 0; i < NUMBER_OF_JUNCTIONS; i++)
        char message[] = {controller, 2, junctionsArray[i], 2 + junctionsArray[i]};
        sendMessage(message);
}

// Wait a control period [2, 15] seconds (controlled by potentiometer).
int getControlPeriod(){
    return  map(analogRead(potentiometer), 2000, 15000, 0, 180);
}

// When the controller wants to send a message he should do: 
void sendMessage(char[] message){ 
    char destination = message[2];
    char operationNumber = message[1];

    // While receiving or sending data the controller’s blue LED must blink.
    startBlinkingBLUE();

    Wire.beginTransmission(destination); // transmit to device 
    Wire.write(message); // sends the message 
    Wire.endTransmission();

    if (operationNumber == 3){
        Wire.requestFrom(destination, 5); // request 5 bytes from slave device when a PING was sent
        int count = 0;
        while(Wire.available()){
            int c = Wire.read();
            count ++;
            if (count == 4)
                checkStatus(c);
            Serial.print(c); // DO SOMETHING WITH THE ACKs ?????????????????
        }
    }
    else{
        Wire.requestFrom(destination, 4); // request 4 bytes from slave device when another message was sent
        // DO SOMETHING WHEN HE RECEIVES A TIME(X) MESSAGE
        while (Wire.available()) { 
            int c = Wire.read(); 
            Serial.print(c); // DO SOMETHING WITH THE ACKs ?????????????????
        } 
    }

    // While receiving or sending data the controller’s blue LED must blink.
    stopBlinkingBLUE();
} 

// While receiving or sending data the controller’s blue LED must blink.
void startBlinkingBLUE(){

}

// While receiving or sending data the controller’s blue LED must blink.
void stopBlinkingBLUE(){

}

void checkStatus(int status){
    int binary[8];
    for (int i = 0; i < 8; i++)
        binary[i] = 0;
    int i = 7;
    while (status > 0) {
        binary[i] = status % 2;
        status = status / 2;
        i--;
    }
    for (int i = 0; i < 8; i++)
        printf("%d", binary[i]);

    // DO SOMETHING WITH THE FAULTY LIGHTS ????????????????

    if (binary[7] == 1)
        timeIsHalved = 1;
}