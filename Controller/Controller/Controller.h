#pragma once
#include "nRF24L01.h"
#include "RF24.h"

#define debug;	//uncomment to get additional debug info over serial

//pinouts
uint8_t xAxisPin = A0;
uint8_t yAxisPin = A1;
uint8_t button1Pin = 4;
uint8_t button2Pin = 5;
uint8_t deadmanPin = 6;
uint8_t vibePin = 9;
uint8_t cePin = 7;
uint8_t csPin = 8;

struct dataStruct {
	int xAxis;	
	int yAxis;
	bool button1;
	bool button2;
	bool deadman;
}oldValues, newValues, heartbeat;

unsigned long prev_time;
const int heartbeat_timeout = 100;		//heartbeat timer 100ms
const int message_frequency = 50;		//frequency messages are sent, time in ms
int deadband = 20;						// joystick centered deadband

//Radio Setup
bool radioNumber = 1;	//Set this radio as radio number 0 or 1
RF24 radio(cePin, csPin);		//Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins ce & cs
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };