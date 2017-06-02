/*
 Name:		Teensy_Vehicle_Controller.ino
 Created:	4/10/2017 11:04:10
 Author:	Travis.rideout
*/

//TODO: implement ID pairing check
//TODO: Gather vehicle info from CAN bus
//TODO: add CAN heartbeat for controller separate from radio

#include "Teensy_Vehicle_Controller.h"

void setup() {
	Serial.begin(115200);

	//Print header
	Serial.println("HDT ROBOTICS");
	Serial.println("WOLF RADIO CAN TRANSLATOR");
	Serial.println("VERSION: 0.1");
	Serial.println();
	
	//Set pin modes	
	pinMode(ledRedPin, OUTPUT);
	pinMode(ledGreenPin, OUTPUT);
	pinMode(ledBluePin, OUTPUT);
	setLED(0, 0, 125);
	
	//initialize objects
	coms.begin();
	can.begin();
	vehicle_imu.begin();	

	//initialize data
	initializeRFMessages();
}

void loop() {	
	if (can.vehicleCANMSg()) {
		//TODO: Collect data from CAN bus and pack into ack msg
	}

	//TODO: Collect gui msg data from CAN bus
	if (coms.receiveMessage(&rxMsg, &pipe)) {
		setLED(0,125,0);

		//TODO: determine what type of ack message to return (fault, GUI, other)
		ackMsg.guiMsg_struct = gui;		
		coms.sendMessage(&ackMsg, controllerAddress);

		parseMessage(&rxMsg);	//TODO: change to passToCAN			
		heartbeatTimer.reset();

		if (fault_state) {
			fault_state = false;
		}

	} else if (heartbeatTimer.check() && !fault_state) { 
		Serial.println("Lost radio signal");
		setLED(0,0,125);
		fault_state = true;
	}

	//Broadcast IMU data to CAN bus at set frequency
	if (imuCANtimer.check() && vehicle_imu.AHRS_active) {
		IMU::imuUnion imudata = {};
		vehicle_imu.readAHRS(&imudata.imuData_struct);
		
		gui.orient[2] = imudata.imuData_struct.yaw;
		gui.orient[0] = imudata.imuData_struct.pitch;
		gui.orient[1] = imudata.imuData_struct.roll;
		gui.imuStatus = imudata.imuData_struct.status;
		
		can.imuCANMsg(imudata);
	}	
}

//initialize RF message constants
void initializeRFMessages() {
	gui.msgType = 'G';
	gui.pairedID = 201;

	fault.msgType = 'F';
	fault.pairedID = 201;
}

void parseMessage(WOLF_COMS::RFMsg_union *msg) {
	Serial.print("Message Type is: ");
	Serial.println((char)msg->heartbeatMsg_struct.msgType);

	switch (msg->heartbeatMsg_struct.msgType){
	case WOLF_COMS::heartbeat:
		can.RFCANMsg(*msg);
		Serial.print("Heartbeat message recieved: ");
		Serial.println(msg->heartbeatMsg_struct.count);
		break;
	case WOLF_COMS::control:
		//pass to CAN bus
		can.RFCANMsg(*msg);
		Serial.print("Control message recieved: ");
		coms.printMessage(msg);
		//print controls message data for debug
		break;
	case WOLF_COMS::userFeedback:
		//pass to CAN bus
		Serial.println("User request message recieved: ");
		//print message data for debug
		break;
	case WOLF_COMS::fault:
		//pass to CAN bus
		Serial.println("Fault message recieved: ");
		//print message data for debug
		break;
	default:
		Serial.println("Invalid message data type");
		break;
	}		
}

void setLED(byte red, byte green, byte blue) {
	analogWrite(ledRedPin, red);
	analogWrite(ledGreenPin, green);
	analogWrite(ledBluePin, blue);
}