#pragma once
#include "WOLF_COMS.h"
#include "OCU.h"

#define debug;	//uncomment to get additional debug info over serial

//Prototypes
void initializeData(WOLF_COMS::controlMsg &data);

//pinouts
//additional pinouts in OCU.h
const uint8_t intPin = 11;	//not sure if interrupts work
const uint8_t csPin = 27;
const uint8_t rstPin = 30;
const uint8_t sck = 12;
const uint8_t mosi = 13;
const uint8_t miso = 14;

//constants
const int heartbeat_frequency = 100;	//100	//frequency heartbeat, time in ms
const int message_frequency = 33;		//33	//frequency messages are sent, time in ms
const int batteryMeasure_frequency = 1000;	//frequency battery percent is measured, time in ms
const int vibe_frequency = 100;				//frequency vibe loop, time in ms

//working variables
unsigned long prev_time = 0;
unsigned long batt_time = 0;
unsigned long count = 0;
byte batt = 100;
uint8_t controllerAddress = 1;
uint8_t vehicleAddress = 2;

WOLF_COMS::controlMsg newValues = {};
WOLF_COMS::RFMsg_union ackMsg = {};
WOLF_COMS::heartbeatMsg heartbeat = {};

//Objects
WOLF_COMS coms(csPin, intPin, rstPin, controllerAddress, false);
OCU ocu;