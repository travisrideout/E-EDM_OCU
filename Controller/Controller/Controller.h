#pragma once
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <EEPROM.h>
#include <TimerOne.h>
#include <Crypto.h>
#include <ChaCha.h>

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
	byte seed[8];
	int xAxis;	
	int yAxis;
	bool button1;
	bool button2;
	bool deadman;
}oldValues, newValues, heartbeat;

struct joystick {
	int center = 512;
	int min = 0;
	int max = 1023;
	int deadband = 40;
}xAxis, yAxis;

struct Encryption {
	byte key[32] = { 0x66, 0x74, 0x7a, 0xc2, 0x11, 0x9a, 0x36, 0x03,
		0x7e, 0x95, 0x85, 0x27, 0x1c, 0xf5, 0xfa, 0x4a,	0xb1,
		0x0d, 0xb6, 0xc5, 0x86, 0xe9, 0xcb, 0x53, 0x23, 0x60, 0x00,
		0x34, 0xae, 0x28, 0x81, 0xd7 };
	size_t keySize = 32;
	uint8_t rounds = 20;
	byte iv[8] = { 0x27, 0xe2, 0x9e, 0x10, 0xbd, 0x14, 0x2d, 0xba };
};

static Encryption const cypher;
ChaCha chacha;

unsigned long prev_time;
unsigned long calibrate_time;
int count = 0;
const int heartbeat_timeout = 100;		//heartbeat timer 100ms
const int message_frequency = 50;		//frequency messages are sent, time in ms
const int calibrate_timeout = 3000;
bool calibrate = false;
const int xJoyMemLoc = 0;
const int yJoyMemLoc = xJoyMemLoc + sizeof(xAxis);

//Vibe interrupt variables
int vibePulses = 0;
int vibeDuration = 100;
int vibeDurationCounter = 100;
int vibeDelay = 100;
int vibeDelayCounter = 100;

//Radio Setup
//bool radioNumber = 1;	//Set this radio as radio number 0 or 1
RF24 radio(cePin, csPin);		//Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins ce & cs
const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };