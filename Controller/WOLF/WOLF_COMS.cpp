#include "WOLF_COMS.h"

//TODO: Programatically set radio PAlevel based on signal strength/lossed packets
//TODO: Implement controller ID's to avoid multiple controller interference

WOLF_COMS::Encryption const WOLF_COMS::cypher;

WOLF_COMS::WOLF_COMS(uint8_t _csPin, uint8_t _intPin, uint8_t _resetPin, uint8_t address, bool _extraDebug)
	: driver(_csPin, _intPin), manager(driver, address), rstPin(_resetPin), extraDebug(_extraDebug) {
}

void WOLF_COMS::begin() {
	//initialize encryption
	chacha.setNumRounds(cypher.rounds);
	chacha.setKey(cypher.key, cypher.keySize);
	chacha.setIV(cypher.iv, chacha.ivSize());

	pinMode(rstPin, INPUT);	//rst is an output on rfm95

	//Manual reset of radio
	Serial.println("Reset Radio");
	pinMode(rstPin, OUTPUT);
	digitalWrite(rstPin, LOW);
	delayMicroseconds(100);
	pinMode(rstPin, INPUT);
	delay(10);	

	//start and configure radio modem manager
	if (!manager.init()) {
		Serial.println("RFM95 init failed");
	}
	manager.setTimeout(retryTimeout);
	manager.setRetries(retries);

	//configure radio driver
	driver.setTxPower(23, false);	//max power
	if (!driver.setFrequency(RF95_FREQ)) {
		Serial.println("setFrequency failed");
	}
	Serial.print("Set Freq to: "); 
	Serial.println(RF95_FREQ);
	driver.setModemConfig(driver.Bw500Cr45Sf128);
	delay(20);

	Serial.println("WOLF_COMS object initialization");
}

bool WOLF_COMS::sendMessage(RFMsg_union *_msg, uint8_t address) {
	encryptMsg(_msg);
	if (!manager.sendtoWait(_msg->msg_bytes, sizeof(_msg->msg_bytes), address)) {
		Serial.println("sendtoWait failed");
		return false;
	} 
	return true;
}

//checks is message is available and gets it if there is one
bool WOLF_COMS::receiveMessage(RFMsg_union *_msg, byte *ackAddress) {
	if (manager.available()) {
		uint8_t len = sizeof(_msg->msg_bytes);		
		if (manager.recvfromAck(_msg->msg_bytes, &len, ackAddress)) {
			decryptMsg(_msg);
			return true;
		}
	}
	return false;
}

//Waits for an ack message, blocks for ackTimeout
bool WOLF_COMS::receiveAckMessage(RFMsg_union *_msg, byte *ackAddress) {
	uint8_t len = sizeof(_msg->msg_bytes);
	if (manager.recvfromAckTimeout(_msg->msg_bytes, &len, ackTimeout, ackAddress)) {
		decryptMsg(_msg);
		return true;
	} 
	return false;
}

void WOLF_COMS::printMessage(RFMsg_union * msg) {
	for (uint8_t i = 0; i < sizeof(msg->msg_bytes); i++) {
		Serial.print(msg->msg_bytes[i]);
		Serial.print(", ");
	}
	Serial.println();
}

void WOLF_COMS::encryptMsg(RFMsg_union *_msg) {
	randomSeed(millis());

	for (uint8_t i = 0; i < 8; i++) {
		_msg->controlMsg_struct.seed[i] = (byte)random(0, 255);
	}

	//print unencrypted message
	if (extraDebug) {
		Serial.println("Unencrypted message: ");
		printMessage(_msg);
	}

	byte msgCrypt[32];
	chacha.setIV(cypher.iv, chacha.ivSize());
	if (!chacha.setCounter(_msg->controlMsg_struct.seed, sizeof(_msg->controlMsg_struct.seed))) {
		Serial.println("Failed to set counter!");
	}
	chacha.encrypt(msgCrypt, _msg->msg_bytes, sizeof(_msg->msg_bytes));
	
	for (uint8_t i = 0; i < 8; i++) {
		msgCrypt[i] = _msg->controlMsg_struct.seed[i];
	}

	//copy encrypted data back into union
	memcpy(_msg->msg_bytes, msgCrypt, sizeof(_msg->msg_bytes));

	//print encrypted message
	if (extraDebug) {
		Serial.println("Encrypted message: ");
		printMessage(_msg);
	}
}

void WOLF_COMS::decryptMsg(RFMsg_union * _msg) {
	//print unencrypted message
	if (extraDebug) {
		Serial.println("Encrypted message: ");
		printMessage(_msg);
	}

	byte seed[8];
	for (uint8_t i = 0; i < 8; i++) {
		seed[i] = _msg->msg_bytes[i];
	}

	chacha.setIV(cypher.iv, chacha.ivSize());
	if (!chacha.setCounter(seed, sizeof(seed))) {
		Serial.println("Failed to set decrypt counter!");
	}
	RFMsg_union decryptedMsg = {};
	chacha.decrypt(decryptedMsg.msg_bytes, _msg->msg_bytes, sizeof(_msg->msg_bytes));
	
	//print unencrypted message
	if (extraDebug) {
		Serial.println("Decrypted message: ");
		printMessage(_msg);
	}

	//copy decrypted data back into union
	memcpy(_msg->msg_bytes, decryptedMsg.msg_bytes, sizeof(decryptedMsg.msg_bytes));
}

WOLF_COMS::~WOLF_COMS() {
}