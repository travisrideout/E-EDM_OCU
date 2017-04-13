#pragma once
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <Crypto.h>
#include <ChaCha.h>
#include <bluefruit.h>
#include <Nffs.h>

#define FILENAME    "/config.txt"
#define debug;	//uncomment to get additional debug info over serial
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096

//pinouts
const uint8_t xAxisPin = A0;
const uint8_t yAxisPin = A1;
const uint8_t battVdcPin = A7;
const uint8_t button1Pin = PIN_A2;
const uint8_t button2Pin = PIN_A3;
const uint8_t cePin = PIN_A4;
const uint8_t csPin = PIN_A5;
const uint8_t sck = 12;
const uint8_t mosi = 13;
const uint8_t miso = 14;
const uint8_t deadmanPin = 15;
const uint8_t vibePin = 16;

struct dataStruct {
	byte seed[8];
	//byte pairedID;
	int xAxis;			//0-1023 based on arduino analog read
	int yAxis;
	bool button1;
	bool button2;
	bool deadman;
	//byte ocuBatt;		//in percent of total 0-100
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

//encryption
static Encryption const cypher;
ChaCha chacha;

//constants
const int heartbeat_frequency = 100;		//frequency heartbeat, time in ms
const int message_frequency = 50;			//frequency messages are sent, time in ms
const int vibe_frequency = 100;				//frequency vibe loop, time in ms
const int calibrate_timeout = 3000;
const int xJoyMemLoc = 0;
const int yJoyMemLoc = xJoyMemLoc + sizeof(xAxis);

//working variables
unsigned long prev_time;
unsigned long calibrate_time;
unsigned long count = 0;
bool calibrate = false;
byte batt = 100;

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

NffsFile file;
