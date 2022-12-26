#include "Wire.h"

// TODO:  Each traffic control light (green, yellow, and red) must have fault detection capability to detect that it is always turned OFF and does not react to its control
// TODO: It must be possible for the controller and any traffic lights, to detect faults of the communications link, or in other traffic lights
// TODO: While receiving or sending data the controller’s blue LED must blink
// TODO: It must be possible to shorten the cycle time by half by pressing the pedestrian button - when the button is pressed the remaining of the cycle time is halved. The reduction affects a single cycle after which the system reverts to its normal operation cycle.

const int controller = 0;

// The controller must be designed as an Arduino UNO with I/O devices attached.
// - 2 LEDs: a red LED to show the controller status (ON or OFF), and a blue LED to indicate activity on the communications bus
#define CONTROLLER_STATUS 1
#define BUS_ACTIVITY 2

// - an ON/OFF button (turn on/turn off)
#define BUTTON 3

// - a potentiometer to select the period of traffic control (enter street #1 → enter street #2 → enter street #3 → enter street #4 → enter street #1)
const int potentiometer = A0;

// The identification of the roundabout entry corresponding to the traffic light is configured by jumpers (or fixed wires) connected to input ports of the Arduino controller – 1, 2, 3, 4 – in ascending order anti-clockwise.  ???????????????
#define JUNCTION_1 8
#define JUNCTION_2 9
#define JUNCTION_3 10
#define JUNCTION_4 11

#define NUMBER_OF_JUNCTIONS 4

byte junctionsArray[] = {JUNCTION_1, JUNCTION_2, JUNCTION_3, JUNCTION_4};

int lastButtonState;
int currentButtonState;


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
                // It must be possible to shorten the cycle time by half by pressing the pedestrian button - when the button is pressed the remaining of the cycle time is halved.          ???????
                delay(getControlPeriod());

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

    if (operationNumber == 3)
        Wire.requestFrom(destination, 5); // request 5 bytes from slave device when a PING was sent
    else
        Wire.requestFrom(destination, 4); // request 4 bytes from slave device when another message was sent
    while (Wire.available()) { 
        char c = Wire.read(); 
        Serial.print(c); // DO SOMETHING WITH THE ACKs ?????????????????
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