/*
Coded by Marjan Olesch
Sketch from Insctructables.com
Open source - do what you want with this code!
*/
#include <Servo.h>

int value = 0; // set values you need to zero
int pin = 10;

Servo firstESC, secondESC; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

void setup() {
    firstESC.attach(pin);    // attached to pin 9 I just do this with 1 Servo
    Serial.begin(9600);    // start serial at 9600 baud
}

bool arm = false;
void loop() {
    //First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions

    if (arm == false)
    {
        arm = true;
        delay(2000);
        firstESC.writeMicroseconds(2000);
        delay(300);
    }

    firstESC.writeMicroseconds(value);

    if (Serial.available())
        value = Serial.parseInt();    // Parse an Integer from Serial
}