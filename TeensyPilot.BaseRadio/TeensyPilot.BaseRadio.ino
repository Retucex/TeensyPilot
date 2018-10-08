#include <RadioHead.h>
#include <RH_RF95.h>

// Serial debug
#define SERIAL_DEBUG

#ifdef SERIAL_DEBUG
#define PLN(x) (Serial.println(x))
#define P(x) (Serial.print(x))
#endif

#ifndef SERIAL_DEBUG
#define PLN(x)
#define P(x)
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
union imuPacket
{
	float imuDat[5];
	uint8_t bytes[20];
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

			P(F("H:"));
			P(buf.imuDat[0]);
			P(F(" P:"));
			P(buf.imuDat[1]);
			P(F(" R:"));
			P(buf.imuDat[2]);
			P(F(" T:"));
			P(buf.imuDat[3]);
			P(F(" A:"));
			PLN(buf.imuDat[4]);
		}
		else
		{
			Serial.println("Receive failed");
		}
	}
}