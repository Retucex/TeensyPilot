/*
Coded by Marjan Olesch
Sketch from Insctructables.com
Open source - do what you want with this code!
*/
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

int value = 0; // set values you need to zero

int pin1 = 10;
int pin2 = 9;
int pin3 = 11;
int pin4 = 6;

int ch3Pin = 4;
int ch3Value = 0;
int ch6Pin = 5;
int ch6Value = 0;

int ledPin = 13;
int buttonPin = 12;

Servo escBL, escBR, escFR, escFL; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned int index = 0;
float pitchErrors[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float rollErrors[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float pitchTarget = 0;
float rollTarget = 0;
float kp = 1;
float ki = 0.2;
float kd = 10;

struct Orientation {
    float Pitch;
    float Roll;
    float Yaw;
};

Orientation GetOrientation(sensors_event_t* event) {
    Orientation orientation;
    orientation.Pitch = event->orientation.y;
    orientation.Roll = event->orientation.z;
    orientation.Yaw = event->orientation.x;
    return orientation;
}

float clamp(float in, float min, float max) {
    if (in < min)
        return min;
    if (in > max)
        return max;
    return in;
}

void setup() {
    escBL.attach(pin1);    // attached to pin 9 I just do this with 1 Servo
    escBR.attach(pin2);
    escFR.attach(pin3);
    escFL.attach(pin4);

    pinMode(ch3Pin, INPUT);
    pinMode(ch6Pin, INPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(buttonPin, INPUT);

    Serial.begin(9600);    // start serial at 9600 baud

    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    delay(1000);

    bno.setExtCrystalUse(true);
}

// 1500 is almost hover

bool arm = false;
bool active = false;
void loop() {
    //First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions

    if (arm == false)
    {
        arm = true;
        escBL.writeMicroseconds(0);
        escBR.writeMicroseconds(0);
        escFR.writeMicroseconds(0);
        escFL.writeMicroseconds(0);
        delay(2000);
    }

    if (active) {
        digitalWrite(ledPin, HIGH);
        active = false;
    }
    else {
        digitalWrite(ledPin, LOW);
        active = true;
    }

    ch3Value = pulseIn(ch3Pin, HIGH);
    ch6Value = pulseIn(ch6Pin, HIGH);

    ///* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);
    Orientation orientation = GetOrientation(&event);

    float pitchError = pitchTarget - orientation.Pitch;
    float pitchErrorSum = 0;

    for (short i = 0; i < 10; i++)
    {
        pitchErrorSum += pitchErrors[i];
    }

    float pidPitch = (kp * pitchError) + (ki * pitchErrorSum) + (kd * (pitchError - pitchErrors[index % 10]));

    float rollError = rollTarget - orientation.Roll;
    float rollErrorSum = 0;

    for (short i = 0; i < 10; i++)
    {
        rollErrorSum += rollErrors[i];
    }

    float pidRoll = (kp * rollError) + (ki * rollErrorSum) + (kd * (rollError - rollErrors[index % 10]));

    index += 1;

    pitchErrors[index % 10] = pitchError;
    rollErrors[index % 10] = rollError;

    if (ch6Value >= 1900) {
        int offset = -500;
        float fr = clamp(ch3Value + offset + pidPitch + pidRoll, 1000, 1600);
        float fl = clamp(ch3Value + offset + pidPitch - pidRoll, 1000, 1600);
        float br = clamp(ch3Value + offset - pidPitch + pidRoll, 1000, 1600);
        float bl = clamp(ch3Value + offset - pidPitch - pidRoll, 1000, 1600);
        escFR.writeMicroseconds(fr);
        escFL.writeMicroseconds(fl);
        escBR.writeMicroseconds(br);
        escBL.writeMicroseconds(bl);

        //Serial.print("fr: ");
        //Serial.print(fr, 4);
        //Serial.print("\tfl: ");
        //Serial.print(fl, 4);
        //Serial.print("\tbr: ");
        //Serial.print(br, 4);
        //Serial.print("\tbl: ");
        //Serial.print(bl, 4);
        //Serial.println("");
    }
    else {
        escBL.writeMicroseconds(0);
        escBR.writeMicroseconds(0);
        escFR.writeMicroseconds(0);
        escFL.writeMicroseconds(0);
    }

    //Serial.print("\tPIDPitch: ");
    //Serial.print(pidPitch);
    //Serial.print("\tPIDRoll: ");
    //Serial.print(pidRoll);
    //Serial.println("");

    /////* Display the floating point data */
    //Serial.print("Pitch: ");
    //Serial.print(orientation.Pitch, 4);
    //Serial.print("\tRoll: ");
    //Serial.print(orientation.Roll, 4);
    //Serial.print("\t\Yaw: ");
    //Serial.print(orientation.Yaw, 4);
    //Serial.println("");
}