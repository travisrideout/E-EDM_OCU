#pragma once

#include "WOLF_COMS.h"
#include "CAN.h"
#include <Metro.h>
#include "IMU.h"

#define debug	//uncomment to get additional debug info over serial

const uint8_t ledBluePin = 35;
const uint8_t ledGreenPin = 36;
const uint8_t ledRedPin = 37;

const uint8_t intPin = 8;
const uint8_t rstPin = 9;
const uint8_t csPin = 10;
const uint8_t mosi = 11;
const uint8_t miso = 12;
const uint8_t sck = 13;

WOLF_COMS::guiMsg gui = {};
WOLF_COMS::faultMsg fault = {};
WOLF_COMS::RFMsg_union rxMsg = {};
WOLF_COMS::RFMsg_union ackMsg = {};

//working variables
int deadband = 5;						// joystick centered deadband
bool fault_state = false;
byte pipe = 0;
uint8_t controllerAddress = 1;
uint8_t vehicleAddress = 2;

//objects
WOLF_COMS coms(csPin, intPin, rstPin, vehicleAddress, false);
CAN can(1000000);	//TODO: need to pick and specify CAN id here
IMU vehicle_imu;
Metro imuCANtimer = Metro(33); //IMU CAN bus broadcast timer, 33ms~=30hz
Metro heartbeatTimer = Metro(220);	//heartbeat timeout timer