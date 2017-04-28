#include "CAN.h"


//k_baud 
CAN::CAN(uint k_buad):CANbus(k_buad) {
}

void CAN::begin() {
	CANbus.begin();
}

bool CAN::nRFCANMsg(WOLF_COMS::nRF24Msg_union msg) {
	switch (msg.nRF24Header_struct.msgType) {
	case WOLF_COMS::control:
		canTXmsg.id = WOLF_COMS::control;
		canTXmsg.len = 8;
		for (size_t i = 0; i < canTXmsg.len; i++) {
			canTXmsg.buf[i] = msg.msg_bytes[headerOffset + i];
		}
		CANbus.write(canTXmsg);
		break;
	case WOLF_COMS::userFeedback:
		canTXmsg.id = WOLF_COMS::userFeedback;
		canTXmsg.len = 3;
		for (size_t i = 0; i < canTXmsg.len; i++) {
			canTXmsg.buf[i] = msg.msg_bytes[headerOffset + i];
		}
		CANbus.write(canTXmsg);
		break;
	default:
		Serial.println("nRF message type undefined - nRFCANMsg");
		return false;
	}
		

	return true;
}

CAN::~CAN() {
}
