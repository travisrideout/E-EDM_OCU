#include "IMU.h"

IMU::IMU() {
}

void IMU::begin() {
	// Initialise AHRS sensor
	if (!bno.begin()) {
		// There was a problem detecting the BNO055 ... check connections
		Serial.print("AHRS not detected ... Check wiring!");
	} else {
		displayAHRSDetails();
		AHRS_active = true;
		bno.setExtCrystalUse(true);
	}
}

void IMU::readAHRS(imuData *imu) {
	sensors_event_t event;
	bno.getEvent(&event);
	uint8_t system, gyro, accel, mag;

	imu->pitch = (int16_t)(event.orientation.x * 100);
	imu->roll = (int16_t)(event.orientation.y * 100);
	imu->yaw = (int16_t)(event.orientation.z * 100);

	bno.getCalibration(&system, &gyro, &accel, &mag);
	imu->status = (byte)(system | gyro << 2 | accel << 4 | mag << 6);
}

void IMU::displayAHRSDetails() {
	sensor_t sensor;
	bno.getSensor(&sensor);
	Serial.println("------------------------------------");
	Serial.print("Sensor:       "); Serial.println(sensor.name);
	Serial.print("Driver Ver:   "); Serial.println(sensor.version);
	Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
	Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
	Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
	Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
	Serial.println("------------------------------------");
	Serial.println("");
}

IMU::~IMU() {
}
