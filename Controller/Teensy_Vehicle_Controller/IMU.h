#pragma once

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMU {
public:
	typedef struct _imuData {
		int16_t pitch;
		int16_t roll;
		int16_t yaw;
		byte status;
	}imuData;

	typedef union _imuUnion {
		imuData imuData_struct;
		uint8_t msg_bytes[7];
	}imuUnion;

	bool AHRS_active = false;

	IMU();
	void begin();
	void readAHRS(imuData*);	//int16_t *x, int16_t *y, int16_t *z, byte *status);	//TODO: Read into a imuData struct
	void displayAHRSDetails();
	~IMU();

private:
	Adafruit_BNO055 bno = Adafruit_BNO055(55);	//IMU Setup

};

