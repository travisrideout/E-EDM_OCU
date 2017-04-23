#pragma once
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <Crypto.h>
#include <ChaCha.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define debug	//uncomment to get additional debug info over serial

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

typedef struct _nRF24Header {
	byte seed[8];
	byte pairedID;
	byte msgType;
}nRF24Header;

typedef struct _controlMsg {
	byte seed[8];
	byte pairedID;
	byte msgType;
	int xAxis;
	int yAxis;
	bool button1;
	bool button2;
	bool deadman;
	byte ocuBatt;		//in percent of total 0-100
}controlMsg;

controlMsg control = {};

typedef struct _heartbeatMsg {
	byte seed[8];
	byte pairedID;
	byte msgType;
	unsigned long count;
}heartbeatMsg;

heartbeatMsg heartbeat = {};

typedef struct _guiMsg {
	byte seed[8];
	byte pairedID;
	byte msgType;
	byte batteryLevel;
	byte fuelLevel;
	byte state;
	int orient[3];
	byte mode;
	byte speed;
	unsigned int mileage;
	unsigned int hours;
	byte accessories;
}guiMsg;

guiMsg gui = {};

typedef struct _faultMsg {
	byte seed[8];
	byte pairedID;
	byte msgType;
	int faultCode;
	byte faultSpecificData[8];
}faultMsg;

faultMsg fault;

typedef union _nRF24Msg_union {
	nRF24Header nRF24Header_struct;
	controlMsg controlMsg_struct;
	heartbeatMsg heartbeatMsg_struct;
	guiMsg guiMsg_struct;
	faultMsg faultMsg_struct;
	uint8_t msg_bytes[32];
}nRF24Msg_union;

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

bool fault_state = false;

struct Encryption {
	byte key[32] = { 0x66, 0x74, 0x7a, 0xc2, 0x11, 0x9a, 0x36, 0x03,
		0x7e, 0x95, 0x85, 0x27, 0x1c, 0xf5, 0xfa, 0x4a,	0xb1,
		0x0d, 0xb6, 0xc5, 0x86, 0xe9, 0xcb, 0x53, 0x23, 0x60, 0x00,
		0x34, 0xae, 0x28, 0x81, 0xd7 };
	size_t keySize = 32;
	uint8_t rounds = 20;
	byte iv[8] = { 0x27, 0xe2, 0x9e, 0x10, 0xbd, 0x14, 0x2d, 0xba };
};

//objects

//Encryption objects
static Encryption const cypher;
ChaCha chacha;

//Radio Setup
RF24 radio(cePin, csPin);		//Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins ce & cs
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };

//IMU Setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//working variables
unsigned long prev_time;
const long heartbeat_timeout = 25000;		//heartbeat timer 
int deadband = 5;						// joystick centered deadband
int faultPrintTimer = 5000;				//error code serial print delay, ms
int clearFaultTimer = 3000;				//error code serial print delay, ms
bool AHRS_active = false;