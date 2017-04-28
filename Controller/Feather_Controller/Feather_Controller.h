#pragma once
#include "WOLF_COMS.h"
#include "OCU.h"

#define debug;	//uncomment to get additional debug info over serial

//Prototypes
void initializeData(WOLF_COMS::controlMsg &data);

//pinouts
//additional pinouts in OCU.h
const uint8_t cePin = PIN_A4;
const uint8_t csPin = PIN_A5;
const uint8_t sck = 12;
const uint8_t mosi = 13;
const uint8_t miso = 14;

//constants
const int heartbeat_frequency = 1000;		//frequency heartbeat, time in ms
const int message_frequency = 50;			//frequency messages are sent, time in ms
const int batteryMeasure_frequency = 1000;	//frequency battery percent is measured, time in ms
const int vibe_frequency = 100;				//frequency vibe loop, time in ms

//working variables
unsigned long prev_time = 0;
unsigned long batt_time = 0;
unsigned long count = 0;
byte batt = 100;


WOLF_COMS::controlMsg newValues = {};
WOLF_COMS::nRF24Msg_union ackMsg = {};
WOLF_COMS::heartbeatMsg heartbeat = {};

//Objects
WOLF_COMS coms(cePin, csPin, true, false);
OCU ocu;