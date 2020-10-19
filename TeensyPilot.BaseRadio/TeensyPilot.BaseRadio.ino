#include <RadioHead.h>
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

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

// Data types

struct Orientation {
    float Pitch;
    float Roll;
    float Yaw;
};

struct Data {
    uint32_t elapsed;
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

union imuPacket
{
    Data dat;
    uint8_t bytes[92];
};

void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);

    while (!Serial);
    Serial.begin(115200);
    delay(100);

    while (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    }
    Serial.println("LoRa radio init OK!");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }
    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

    // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

    // The default transmitter power is 13dBm, using PA_BOOST.
    // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
    // you can set transmitter powers from 5 to 23 dBm:
    rf95.setTxPower(23, false);

    PLN("");
    P("t,fr,fl,br,bl,pitch,roll,yaw,pp,pi,pd,rp,ri,rd,ch1,ch2,ch3,ch4,ch5,ch6");
    PLN("");
}

void loop()
{
    if (rf95.available())
    {
        // Should be a message for us now
        imuPacket buf;
        uint8_t len = sizeof(buf);
        memset(buf.bytes, 0, sizeof buf);

        if (rf95.recv(buf.bytes, &len))
        {
            digitalWrite(LED, HIGH);

            P(buf.dat.elapsed); P(",");
            PFLOAT(buf.dat.frOutput, 4); P(",");
            PFLOAT(buf.dat.flOutput, 4); P(",");
            PFLOAT(buf.dat.brOutput, 4); P(",");
            PFLOAT(buf.dat.blOutput, 4); P(",");
            PFLOAT(buf.dat.orientation.Pitch, 4); P(",");
            PFLOAT(buf.dat.orientation.Roll, 4); P(",");
            PFLOAT(buf.dat.orientation.Yaw, 4); P(",");
            PFLOAT(buf.dat.pPitch, 4); P(",");
            PFLOAT(buf.dat.iPitch, 4); P(",");
            PFLOAT(buf.dat.dPitch, 4); P(",");
            PFLOAT(buf.dat.pRoll, 4); P(",");
            PFLOAT(buf.dat.iRoll, 4); P(",");
            PFLOAT(buf.dat.dRoll, 4); P(",");
            PFLOAT(buf.dat.pYaw, 4); P(",");
            PFLOAT(buf.dat.iYaw, 4); P(",");
            PFLOAT(buf.dat.dYaw, 4); P(",");
            P(buf.dat.ch1); P(",");
            P(buf.dat.ch2); P(",");
            P(buf.dat.ch3); P(",");
            P(buf.dat.ch4); P(",");
            P(buf.dat.ch5); P(",");
            P(buf.dat.ch6); PLN("");
        }
        else
        {
            Serial.println("Receive failed");
        }
    }
}