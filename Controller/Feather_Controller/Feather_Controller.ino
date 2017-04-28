/*
 Name:		Feather_Controller.ino
 Created:	4/5/2017 09:57:14
 Author:	Travis.rideout
*/

//TODO: add BLE for operator GUI
//TODO: create error handler

//TODO: utilize sleep modes on no user interaction timeouts to save battery

#include "Feather_Controller.h"

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	
	//Print header
	Serial.println("HDT ROBOTICS");
	Serial.println("E-EDM FEATHER CONTROLLER");
	Serial.println("VERSION: 0.1");
	Serial.println(" ");
	
	coms.begin();
	ocu.begin();
	batt = ocu.readBatt();

	//initialize data	
	initializeData(newValues);
			
	Scheduler.startLoop(vibeTask);

	prev_time = millis();

	WOLF_COMS::nRF24Msg_union send = {};
	send.heartbeatMsg_struct = heartbeat;
	send.heartbeatMsg_struct.count = count;
	send.heartbeatMsg_struct.msgType = 'H';
	while (!coms.establishConnectionTX(&send)) {
		delay(3000);
	}
}

//realtime loop, not delayed
void loop() {		
	if (ocu.calibrateRequested()) {
		ocu.calibrateJoysticks();
	}	

	//All nRF24 calls should be from here to avoid overlap
	if (millis() - prev_time > message_frequency && ocu.readInputs(&newValues)) {
		prev_time = millis();
		WOLF_COMS::nRF24Msg_union send = {};
		newValues.ocuBatt = batt; 
		send.controlMsg_struct = newValues;
		if (!coms.sendMessage(&send)) {
			WOLF_COMS::nRF24Msg_union send = {};
			send.heartbeatMsg_struct = heartbeat;
			send.heartbeatMsg_struct.count = count;
			send.heartbeatMsg_struct.msgType = 'H';
			while (!coms.establishConnectionTX(&send)) {
				delay(3000);
			}
		}
	} else if (millis() - prev_time > heartbeat_frequency) {
		prev_time = millis();
		Serial.print("sending heartbeat message: ");
		count++;
		Serial.println(count);		
		WOLF_COMS::nRF24Msg_union send = {};
		send.heartbeatMsg_struct = heartbeat;
		send.heartbeatMsg_struct.count = count;
		send.heartbeatMsg_struct.msgType = 'H';
		if (!coms.sendMessage(&send)) {
			while (!coms.establishConnectionTX(&send)) {
				delay(3000);
			}
		} 		
	}

	//read battery voltage on delay
	if (millis() - batt_time > batteryMeasure_frequency) {
		batt_time = millis();
		batt = ocu.readBatt();
	}

	//read ack messages
	//TODO: sort and parse messages, shuttle to BT
	if (coms.receiveAckMessage(&ackMsg)) {
		Serial.print("Ack msg type is: ");
		Serial.println((char)ackMsg.nRF24Header_struct.msgType);

		Serial.print("pitch = ");
		Serial.print((float)ackMsg.guiMsg_struct.orient[0]/100.0f);
		Serial.print(", roll = ");
		Serial.print((float)ackMsg.guiMsg_struct.orient[1]/100.0f);
		Serial.print(", heading = ");
		Serial.println((float)ackMsg.guiMsg_struct.orient[2] / 100.0f);
	}
}

//loop for vibe motor, runs on 100ms delays
void vibeTask() {
	ocu.vibe();
	delay(vibe_frequency);
}

//initialize data struct to zero values
//TODO: is this needed? only send control if situation valid else heartbeat
void initializeData(WOLF_COMS::controlMsg &data) {
	data.pairedID = 201;
	data.msgType = 'C';
	data.xAxis = 512;
	data.yAxis = 512;
	data.button1 = true;
	data.button2 = true;
	data.deadman = true;
	data.ocuBatt = 100;
}