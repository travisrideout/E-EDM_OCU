#include "CAN.h"


//k_baud 
CAN::CAN(uint k_buad):CANbus(k_buad) {
}

void CAN::begin() {
	CANbus.begin();
}

bool CAN::RFCANMsg(WOLF_COMS::RFMsg_union msg) {
	switch (msg.nRF24Header_struct.msgType) {
	case WOLF_COMS::control:
		canTXmsg.id = WOLF_COMS::control;
		canTXmsg.len = sizeof(WOLF_COMS::controlMsg) - sizeof(WOLF_COMS::nRF24Header);	// 8;
		for (size_t i = 0; i < canTXmsg.len; i++) {
			canTXmsg.buf[i] = msg.msg_bytes[sizeof(WOLF_COMS::nRF24Header) + i];
		}
		CANbus.write(canTXmsg);
		break;
	case WOLF_COMS::heartbeat:
		canTXmsg.id = WOLF_COMS::heartbeat;
		canTXmsg.len = sizeof(WOLF_COMS::heartbeatMsg) - sizeof(WOLF_COMS::nRF24Header);	// 8;
		for (size_t i = 0; i < canTXmsg.len; i++) {
			canTXmsg.buf[i] = msg.msg_bytes[sizeof(WOLF_COMS::nRF24Header) + i];
		}
		CANbus.write(canTXmsg);
		break;
	case WOLF_COMS::userFeedback:
		canTXmsg.id = WOLF_COMS::userFeedback;
		canTXmsg.len = sizeof(WOLF_COMS::userFeedbackMsg) - sizeof(WOLF_COMS::nRF24Header); //3;
		for (size_t i = 0; i < canTXmsg.len; i++) {
			canTXmsg.buf[i] = msg.msg_bytes[sizeof(WOLF_COMS::nRF24Header) + i];
		}
		CANbus.write(canTXmsg);
		break;
	default:
		Serial.println("nRF message type undefined - nRFCANMsg");
		return false;
	}
		

	return true;
}

void CAN::imuCANMsg(IMU::imuUnion data) {
	canTXmsg.id = WOLF_COMS::imu;
	canTXmsg.len = 7; //manually defining length as sizeof(imuData) returned 8 due to padding
	for (size_t i = 0; i < canTXmsg.len; i++) {
		canTXmsg.buf[i] = data.msg_bytes[i];
	}
	CANbus.write(canTXmsg);
}

//TODO: parse messages and pack for return
bool CAN::vehicleCANMSg() {
	if (CANbus.available()) {
		return true;
	}

	return false;
}


CAN::~CAN() {
}
