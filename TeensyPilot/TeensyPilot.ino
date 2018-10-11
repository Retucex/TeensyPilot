#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <MahonyAHRS.h>
//#include <Adafruit_AHRS\Madgwick.h>

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

// Radio
// Pins
#define RFM95_CS 10
#define RFM95_RST 24
#define RFM95_INT 2

#define RF95_FREQ 434.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// IMU
#define SAMP_PER_SEC 70

float mag_offsets[3] = { 30.05F, -3.03F, -11.19F };

float mag_softiron_matrix[3][3] = { { 1.032, -0.05, -0.025 },
									{ -0.05, 0.959, 0.0 },
									{ -0.025, 0.02, 1.014 } };

float mag_field_strength = 66.45F;

Adafruit_L3GD20_Unified       gyro(20);
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
Adafruit_BMP085_Unified       bmp(18001);

Mahony filter;
//Madgwick filter;

// Data types

// a*: accel in m/s
// r*: rotation in rad/s
// attitude in degrees
// temp in celsius
// altitude in m
struct imuData
{
	float ax;
	float ay;
	float az;
	float rx;
	float ry;
	float rz;
	float pitch;
	float roll;
	float yaw;
	float temperature;
	float altitude;
};

union radioPacket
{
	imuData dat;
	uint8_t bytes[RH_RF95_MAX_MESSAGE_LEN];
};

// Prototypes
void GetIMUData(imuData& dat);

void setup()
{
	// Stabilize
	delay(100);

#ifdef SERIAL_DEBUG
	while (!Serial);
	Serial.begin(115200);
	delay(100);
#endif

	InitRadio();
	InitSensors();

	// Stabilize
	delay(100);
}

void loop()
{
	radioPacket imu;
	imuData dat;
	GetIMUData(dat);

	Transmit(imu.bytes, 44);
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

void InitSensors()
{
	if (!accel.begin())
	{
		PLN(F("No LSM303 (accel) detected"));
		while (1);
	}
	PLN(F("LSM303 (accel) init OK"));

	if (!gyro.begin())
	{
		PLN(F("No L3GD20 (gyro) detected"));
		while (1);
	}
	PLN(F("L3GD20 (gyro) init OK"));

	if (!mag.begin())
	{
		PLN(F("No LSM303 (mag) detected"));
		while (1);
	}
	PLN(F("LSM303 (mag) init OK"));

	if (!bmp.begin())
	{
		PLN(F("No BMP180 (baro) detected"));
		while (1);
	}
	PLN(F("BMP180 (baro) init OK"));

	filter.begin(SAMP_PER_SEC);
}

void Transmit(uint8_t *radiopacket, int size)
{
	P(F("Sending: "));
	PLN((char*)radiopacket);
	if (rf95.send((uint8_t *)radiopacket, size))
	{
		PLN(F("Success, queued for transmission"));
	}
	else
	{
		PLN(F("Failure, CAD timeout"));
	}
}

// TODO Watch out for buf, could be empty when returned. Look at malloc. Don't forget to free
uint8_t* Receive()
{
	uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
	uint8_t len = sizeof(buf);

	PLN(F("Waiting for reply: "));
	if (rf95.waitAvailableTimeout(10))
	{
		if (rf95.recv(buf, &len))
		{
			PLN(F("Got reply: "));
			PLN((char*)buf);
			PLN(F("RSSI: "));
			PLN(rf95.lastRssi());
			return buf;
		}
		else
		{
			PLN(F("Receive failed"));
		}
	}
	else
	{
		PLN(F("WaitAvailableTimeout() Timed out"));
	}
}

void GetIMUData(imuData& dat)
{
	sensors_event_t gyro_event;
	sensors_event_t accel_event;
	sensors_event_t mag_event;
	sensors_event_t bmp_event;

	// Get new data samples
	gyro.getEvent(&gyro_event);
	accel.getEvent(&accel_event);
	mag.getEvent(&mag_event);
	bmp.getEvent(&bmp_event);

	// Apply mag offset compensation (base values in uTesla)
	float x = mag_event.magnetic.x - mag_offsets[0];
	float y = mag_event.magnetic.y - mag_offsets[1];
	float z = mag_event.magnetic.z - mag_offsets[2];

	// Apply mag soft iron error compensation
	float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
	float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
	float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

	// rad/s to deg/s
	float gx = gyro_event.gyro.x * 57.2958F;
	float gy = gyro_event.gyro.y * 57.2958F;
	float gz = gyro_event.gyro.z * 57.2958F;

	// Update the filter
	filter.update(gx, gy, gz,
		accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
		mx, my, mz);

	float temperature, altitude;
	if (bmp_event.pressure)
	{
		bmp.getTemperature(&temperature);
		altitude = bmp.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,
			bmp_event.pressure,
			temperature);
	}

	dat.pitch = filter.getPitch();
	dat.roll = filter.getRoll();
	dat.yaw = filter.getYaw();
	dat.ax = accel_event.acceleration.x;
	dat.ay = accel_event.acceleration.y;
	dat.az = accel_event.acceleration.z;
	dat.rx = gyro_event.acceleration.x;
	dat.ry = gyro_event.acceleration.y;
	dat.rz = gyro_event.acceleration.z;
	dat.temperature = temperature;
	dat.altitude = altitude;

	P(millis());
	P(F(" - H:"));
	P(dat.yaw);
	P(F(" P:"));
	P(dat.pitch);
	P(F(" R:"));
	P(dat.roll);
	P(F(" T:"));
	P(dat.temperature);
	P(F(" A:"));
	P(dat.altitude);
	P(F(" aX:"));
	P(dat.ax);
	P(F(" aY:"));
	P(dat.ay);
	P(F(" aZ:"));
	P(dat.az);
	P(F(" rX:"));
	P(dat.rx);
	P(F(" rY:"));
	P(dat.ry);
	P(F(" rZ:"));
	PLN(dat.rz);
}