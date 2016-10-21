#pragma once
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define debug;	//uncomment to get additional debug info over serial

//pinouts
uint8_t leftMotorSpeedPin = 4;
uint8_t leftMotorBrakePin = 5;
uint8_t leftMotorForwardPin = 22;
uint8_t leftMotorReversePin = 23;
uint8_t leftMotorInterlockPin = 24;

uint8_t rightMotorSpeedPin = 6;
uint8_t rightMotorBrakePin = 7;
uint8_t rightMotorForwardPin = 25;
uint8_t rightMotorReversePin = 26;
uint8_t rightMotorInterlockPin = 27;

uint8_t clearFaultPin = 28;

uint8_t ledRedPin = 13;
uint8_t ledGreenPin = 12;
uint8_t ledBluePin = 11;

uint8_t cePin = 49;
uint8_t csPin = 53;

struct dataStruct {
	int xAxis;
	int yAxis;
	bool button1;
	bool button2;
	bool deadman;
}newValues;

struct motor {
	int speed;		//0-255 pwm
	int brake;
	bool direction;	//0=forward, 1=reverse
	bool interlock;
}leftMotor, rightMotor;

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

unsigned long prev_time;
const long heartbeat_timeout = 150;		//heartbeat timer 150ms
int deadband = 5;						// joystick centered deadband
int faultPrintTimer = 5000;				//error code serial print delay, ms
int clearFaultTimer = 3000;				//error code serial print delay, ms

//Radio Setup
//bool radioNumber = 0;	//Set this radio as radio number 0 or 1
RF24 radio(cePin, csPin);		//Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins ce & cs
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };
