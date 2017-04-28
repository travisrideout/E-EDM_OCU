#pragma once

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "WOLF_COMS.h"
#include "CAN.h"

#define debug	//uncomment to get additional debug info over serial

//prototypes
void parseMessage(WOLF_COMS::nRF24Msg_union *msg);

//pinouts
//const uint8_t clearFaultPin = 28;

const uint8_t ledBluePin = 35;
const uint8_t ledGreenPin = 36;
const uint8_t ledRedPin = 37;

const uint8_t cePin = 9;
const uint8_t csPin = 10;
const uint8_t mosi = 11;
const uint8_t miso = 12;
const uint8_t sck = 13;

WOLF_COMS::controlMsg control = {};
WOLF_COMS::heartbeatMsg heartbeat = {};
WOLF_COMS::guiMsg gui = {};
WOLF_COMS::faultMsg fault = {};

struct leds {
	int red;	//use 0-125
	int green;	//use 0-125
	int blue;	//use 0-125
}led;

enum faultCodes {
	No_Fault = 0,
	Input_Out_Of_Range = 1,
	Loss_Of_Signal = 2,
	Unable_To_Establish_Connection = 3
} faultCode;

//objects
WOLF_COMS coms(cePin, csPin, false, false);
Adafruit_BNO055 bno = Adafruit_BNO055(55);	//IMU Setup
CAN can(500000);

//working variables
unsigned long prev_time;
const long heartbeat_timeout = 25000;		//heartbeat timer 
int deadband = 5;						// joystick centered deadband
int faultPrintTimer = 5000;				//error code serial print delay, ms
int clearFaultTimer = 3000;				//error code serial print delay, ms
bool AHRS_active = false;
bool fault_state = false;