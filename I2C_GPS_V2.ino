/*
    Name:       I2C_GPS_V2.ino
    Created:	2 May 2019 
    Author:     John Semmens

	SLAVE I2C Interface for SERIAL GPS using Arduino Pro Mini 328P
	----------------------------------------------------------------
	Interface for converting a serial GPS to allow it to be used on an I2C bus.
	This uses the Arduino Pro Mini 328P running at 16MHz and 3V3. (Yes, i know its outside of specification!!!).
	This simple sketch uses the very excellent TinyGPS++ to parse the GPS serial data and load the GPS object.
	Then we copy individual field values into our GPS structure which is mapped to linear Data buffer using UNION statement.
	The Data buffer is then written out to the I2C interface in response to an I2C request.

	Wiring:
	The Tx Pin from the GPS is connected to the RX pin of the Arduino Pro Mini 328P using a 1k resistor.
	This provides isolation to allow the Arduino to be programmed over the serial interface while the GPS is connected.
	If this were a direct connection, serial programming of the Arduino would not be possible without disconnecting the GPS.
*/

#include <SPI.h>
#include <Wire.h>
#include "TinyGPS++.h"

// The TinyGPS++ object
TinyGPSPlus gps;

static const uint32_t GPSBaud = 9600;

#define  SLAVE_ADDRESS           0x29  //slave I2C address: 0x01 to 0x7F
#define  REG_MAP_SIZE            18 

// GPS variables
typedef union {
	struct {
		long Lat, Long;					// 2 x 4 bytes
		uint8_t Year, Month, Day;		// 3 x 1 bytes
		uint8_t Hour, Minute, Second;	// 3 x 1 bytes
		int COG, SOG;					// 2 x 2 bytes SOG is in m/s
	};
	uint8_t Data[REG_MAP_SIZE];			//  = 18 bytes
} buffer_t;

buffer_t gps_data,buf;

boolean newDataAvailable = false;

void setup()
{
	Wire.begin(SLAVE_ADDRESS);
	Wire.onRequest(requestEvent);

	Serial.begin(GPSBaud);
}

void loop()
{
	while (Serial.available() > 0)
	{
		if (gps.encode(Serial.read()))
		{
			LoadRegisters();
			newDataAvailable = true;
		}
	}
}

void requestEvent()
{
	if (newDataAvailable)
	{
		for (int c = 0; c < (REG_MAP_SIZE); c++) 
		{
			buf.Data[c] = gps_data.Data[c]; 
		}
	}
	newDataAvailable = false;

	Wire.write(buf.Data, REG_MAP_SIZE);
}

void LoadRegisters()
{
	gps_data.Lat = gps.location.lat() * 10000000UL;
	gps_data.Long = gps.location.lng() * 10000000UL;
	gps_data.COG = gps.course.deg() * 100;
	gps_data.SOG = gps.speed.mps() * 100; // m/s

	gps_data.Year = gps.date.year()-2000;
	gps_data.Month = gps.date.month();
	gps_data.Day = gps.date.day();
	gps_data.Hour = gps.time.hour();
	gps_data.Minute = gps.time.minute();
	gps_data.Second = gps.time.second();

	// Copy gps buffer to buf buffer
	for (int i = 0; i<REG_MAP_SIZE; i++)
		buf.Data[i] = gps_data.Data[i];
}