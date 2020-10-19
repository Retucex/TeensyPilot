#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <RH_RF95.h>

// Serial debug
#define SERIAL_DEBUG

#ifdef SERIAL_DEBUG
#define PLN(x) (Serial.println(x))
#define P(x) (Serial.print(x))
#define PFLOAT(x, y) (Serial.print(x, y))
#endif

#ifndef SERIAL_DEBUG
#define PLN(x)
#define P(x)
#define PFLOAT(x, y)
#endif

// PINS
#define BACK_LEFT_MOTOR_SIGNAL 36
#define BACK_RIGHT_MOTOR_SIGNAL 37
#define FRONT_LEFT_MOTOR_SIGNAL 28
#define FRONT_RIGHT_MOTOR_SIGNAL 29

#define CHANNEL1_SIGNAL 9 // Roll
#define CHANNEL2_SIGNAL 8 // Pitch
#define CHANNEL3_SIGNAL 7 // Speed
#define CHANNEL4_SIGNAL 6 // Yaw
#define CHANNEL5_SIGNAL 5 // Mode
#define CHANNEL6_SIGNAL 4 // Arm

#define RFM95_CS 24
#define RFM95_RST 25
#define RFM95_INT 26

// SETTINGS
#define RF95_FREQ 434.0

const float blBias = 1;
const float brBias = 1;
const float flBias = 1;
const float frBias = 1;

const float kp = 0.5;
const float ki = 0.5;
const float kd = 5;
const float offset = 0;
const float clampMin = 1000;
const float clampMax = 2000;

// OBJECTS
Servo escBL, escBR, escFR, escFL;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// INPUTS
unsigned int tindex = 0;

int ch1Value = 0;
int ch2Value = 0;
int ch3Value = 0;
int ch4Value = 0;
int ch5Value = 0;
int ch6Value = 0;

int ch3Input[5] = { 0, 0, 0, 0, 0 };
int ch3Avg = 0;

float pitchErrors[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float rollErrors[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float yawErrors[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float pitchTarget = 0;
float rollTarget = 0;
float yawTarget = 0;

struct Orientation {
    float Pitch;
    float Roll;
    float Yaw;
};

struct Data {
    int elapsed;
    float flOutput;
    float frOutput;
    float blOutput;
    float brOutput;
    Orientation orientation;
    float pPitch;
    float iPitch;
    float dPitch;
    float pRoll;
    float iRoll;
    float dRoll;
    float pYaw;
    float iYaw;
    float dYaw;
    float ch1;
    float ch2;
    float ch3;
    float ch4;
    float ch5;
    float ch6;
};

union radioPacket
{
    Data dat;
    uint8_t bytes[92];
};

Orientation GetOrientation() {
    // Get a new sensor event
    sensors_event_t event;
    bno.getEvent(&event);
    Orientation orientation;
    orientation.Pitch = event.orientation.y;
    orientation.Roll = event.orientation.z;
    orientation.Yaw = event.orientation.x;
    return orientation;
}

float clamp(float in, float min, float max) {
    if (in < min)
        return min;
    if (in > max)
        return max;
    return in;
}

void InitRadio()
{
    while (!rf95.init())
    {
        PLN(F("LoRa radio init failed"));
        while (1);
    }
    PLN(F("LoRa radio init OK"));

    if (!rf95.setFrequency(RF95_FREQ)) {
        PLN(F("setFrequency failed"));
        while (1);
    }
    P(F("Set Freq to: "));
    PLN(RF95_FREQ);

    rf95.setTxPower(23);
}

void getChannelInputs() {
    ch3Value = pulseIn(CHANNEL3_SIGNAL, HIGH, 20000);
    ch3Input[tindex % 5] = ch3Value;
    ch3Value = 0;
    for (int i = 0; i < 5; i++)
    {
        ch3Value += ch3Input[i];
    }
    ch3Value /= 5;

    if (tindex % 5 == 0) {
        ch1Value = pulseIn(CHANNEL1_SIGNAL, HIGH, 20000);
    }
    if (tindex % 5 == 1) {
        ch2Value = pulseIn(CHANNEL2_SIGNAL, HIGH, 20000);
    }
    if (tindex % 5 == 2) {
        ch4Value = pulseIn(CHANNEL4_SIGNAL, HIGH, 20000);
    }
    if (tindex % 5 == 3) {
        ch5Value = pulseIn(CHANNEL5_SIGNAL, HIGH, 20000);
    }
    if (tindex % 5 == 4) {
        ch6Value = pulseIn(CHANNEL6_SIGNAL, HIGH, 20000);
    }
}

void Transmit(uint8_t* radiopacket, int size)
{
    //P(F("Sending: "));
    //PLN((char*)radiopacket);
    if (rf95.send((uint8_t*)radiopacket, size))
    {
        //PLN(F("Success, queued for transmission"));
    }
    else
    {
        PLN(F("Failure, CAD timeout"));
    }
}

void setup() {
    delay(100);
#ifdef SERIAL_DEBUG
    Serial.begin(2000000);    // start serial at 115200  baud
#endif // SERIAL_DEBUG

    escBL.attach(BACK_LEFT_MOTOR_SIGNAL);    // attached to pin 9 I just do this with 1 Servo
    escBR.attach(BACK_RIGHT_MOTOR_SIGNAL);
    escFR.attach(FRONT_RIGHT_MOTOR_SIGNAL);
    escFL.attach(FRONT_LEFT_MOTOR_SIGNAL);

    pinMode(CHANNEL1_SIGNAL, INPUT);
    pinMode(CHANNEL2_SIGNAL, INPUT);
    pinMode(CHANNEL3_SIGNAL, INPUT);
    pinMode(CHANNEL4_SIGNAL, INPUT);
    pinMode(CHANNEL5_SIGNAL, INPUT);
    pinMode(CHANNEL6_SIGNAL, INPUT);

    if (!bno.begin())
    {
        PLN(F("Unable to start IMU"));
        while (1);
    }

    InitRadio();

    delay(1000);
    bno.setExtCrystalUse(true);

    // Arming
    escBL.writeMicroseconds(0);
    escBR.writeMicroseconds(0);
    escFR.writeMicroseconds(0);
    escFL.writeMicroseconds(0);
    delay(2000);
}

bool setYaw = true;
bool header = false;
void loop() {
    Orientation orientation = GetOrientation();
    getChannelInputs();

    rollTarget = -map(ch1Value, 1000, 2000, -45, 45);
    pitchTarget = -map(ch2Value, 1000, 2000, -45, 45);

    if (setYaw) {
        setYaw = false;
    }

    // PID
    float pitchError = pitchTarget - orientation.Pitch;
    float pitchErrorSum = 0;

    for (short i = 0; i < 10; i++)
    {
        pitchErrorSum += pitchErrors[i];
    }

    float pPitch = kp * pitchError;
    float iPitch = ki * pitchErrorSum;
    float dPitch = kd * (pitchError - pitchErrors[tindex % 10]);
    float pidPitch = pPitch + iPitch + dPitch;

    float rollError = rollTarget - orientation.Roll;
    float rollErrorSum = 0;

    for (short i = 0; i < 10; i++)
    {
        rollErrorSum += rollErrors[i];
    }

    float pRoll = kp * rollError;
    float iRoll = ki * rollErrorSum;
    float dRoll = kd * (rollError - rollErrors[tindex % 10]);
    float pidRoll = pRoll + iRoll + dRoll;

    float yawError = yawTarget - orientation.Yaw;
    if (yawError > 180)
        yawError -= 360;
    else if (yawError < -180)
        yawError += 360;

    float yawErrorSum = 0;

    for (short i = 0; i < 10; i++)
    {
        yawErrorSum += yawErrors[i];
    }

    float pYaw = kp * yawError;
    float iYaw = ki * yawErrorSum;
    float dYaw = kd * (yawError - yawErrors[tindex % 10]);
    float pidYaw = pYaw + iYaw + dYaw;

    // ESC Values
    float fr = clamp((ch3Value + offset + pidPitch + pidRoll + pidYaw) * frBias, clampMin, clampMax);
    float fl = clamp((ch3Value + offset + pidPitch - pidRoll - pidYaw) * flBias, clampMin, clampMax);
    float br = clamp((ch3Value + offset - pidPitch + pidRoll - pidYaw) * brBias, clampMin, clampMax);
    float bl = clamp((ch3Value + offset - pidPitch - pidRoll + pidYaw) * blBias, clampMin, clampMax);

    // ESC Outputs
    if (ch6Value >= 1900) {
        escFR.writeMicroseconds(fr);
        escFL.writeMicroseconds(fl);
        escBR.writeMicroseconds(br);
        escBL.writeMicroseconds(bl);
    }
    else {
        escBL.writeMicroseconds(0);
        escBR.writeMicroseconds(0);
        escFR.writeMicroseconds(0);
        escFL.writeMicroseconds(0);
    }

    // Increase Index, add new error
    tindex += 1;

    pitchErrors[tindex % 10] = pitchError;
    rollErrors[tindex % 10] = rollError;
    yawErrors[tindex % 10] = yawError;

    if (tindex % 10 == 0) {
        radioPacket pack;
        pack.dat.elapsed = millis();
        pack.dat.blOutput = bl;
        pack.dat.brOutput = br;
        pack.dat.flOutput = fl;
        pack.dat.frOutput = fr;
        pack.dat.orientation = orientation;
        pack.dat.pPitch = pPitch;
        pack.dat.iPitch = iPitch;
        pack.dat.dPitch = dPitch;
        pack.dat.pRoll = pRoll;
        pack.dat.iRoll = iRoll;
        pack.dat.dRoll = dRoll;
        pack.dat.pYaw = pYaw;
        pack.dat.iYaw = iYaw;
        pack.dat.dYaw = dYaw;
        pack.dat.ch1 = ch1Value;
        pack.dat.ch2 = ch2Value;
        pack.dat.ch3 = ch3Value;
        pack.dat.ch4 = ch4Value;
        pack.dat.ch5 = ch5Value;
        pack.dat.ch6 = ch6Value;

        Transmit(pack.bytes, sizeof(pack.bytes));
    }

    // SERIAL_DEBUG
    if (!header) {
        header = true;
        PLN("");
        P("t,fr,fl,br,bl,pitch,roll,yaw,pp,pi,pd,rp,ri,rd,ch1,ch2,ch3,ch4,ch5,ch6");
        PLN("");
    }
    P(millis()); P(",");
    PFLOAT(fr, 4); P(",");
    PFLOAT(fl, 4); P(",");
    PFLOAT(br, 4); P(",");
    PFLOAT(bl, 4); P(",");
    PFLOAT(orientation.Pitch, 4); P(",");
    PFLOAT(orientation.Roll, 4); P(",");
    PFLOAT(orientation.Yaw, 4); P(",");
    PFLOAT(pPitch, 4); P(",");
    PFLOAT(iPitch, 4); P(",");
    PFLOAT(dPitch, 4); P(",");
    PFLOAT(pRoll, 4); P(",");
    PFLOAT(iRoll, 4); P(",");
    PFLOAT(dRoll, 4); P(",");
    PFLOAT(pYaw, 4); P(",");
    PFLOAT(iYaw, 4); P(",");
    PFLOAT(dYaw, 4); P(",");
    P(ch1Value); P(",");
    P(ch2Value); P(",");
    P(ch3Value); P(",");
    P(ch4Value); P(",");
    P(ch5Value); P(",");
    P(ch6Value); PLN("");
}