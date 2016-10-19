#pragma once
#include "nRF24L01.h"
#include "RF24.h"

#define debug;	//uncomment to get additional debug info over serial

//pinouts
uint8_t leftMotorSpeedPin = 2;
uint8_t leftMotorBrakePin = 3;
uint8_t leftMotorForwardPin = 4;
uint8_t leftMotorReversePin = 5;
uint8_t leftMotorInterlockPin = 6;
uint8_t cePin = 7;
uint8_t csPin = 8;
uint8_t rightMotorSpeedPin = 9;
uint8_t rightMotorBrakePin = 10;
uint8_t rightMotorForwardPin = 11;
uint8_t rightMotorReversePin = 12;
uint8_t rightMotorInterlockPin = 13;
uint8_t clearFaultPin = 22;


struct dataStruct {
	int xAxis;
	int yAxis;
	bool button1;
	bool button2;
	bool deadman;
}newValues;

struct motor {
	int speed;	//0-100%
	int brake;
	bool direction;	//0=forward, 1=reverse
	bool interlock;
}leftMotor, rightMotor;

enum faultCodes {
	No_Fault = 0,
	Input_Out_Of_Range = 1,
	Loss_Of_Signal = 2
} faultCode;

bool fault_state = false;

unsigned long prev_time;
const long heartbeat_timeout = 150;		//heartbeat timer 150ms
int deadband = 20;						// joystick centered deadband
int faultPrintTimer = 5000;				//error code serial print delay, ms
int clearFaultTimer = 3000;				//error code serial print delay, ms

//Radio Setup
bool radioNumber = 0;	//Set this radio as radio number 0 or 1
RF24 radio(cePin, csPin);		//Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins ce & cs
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };
