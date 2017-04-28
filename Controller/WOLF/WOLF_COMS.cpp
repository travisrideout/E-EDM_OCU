#include "WOLF_COMS.h"

//TODO: Programatically set radio PAlevel based on signal strength/lossed packets
//TODO: frequency hopping
//TODO: Frequency scan on startup
//TODO: Implement controller ID's to avoid multiple controller interference

WOLF_COMS::Encryption const WOLF_COMS::cypher;

//cePin, csPin, primary Radio?, extra debug info
WOLF_COMS::WOLF_COMS(uint8_t _cePin, uint8_t _csPin, bool primaryRadio, bool _extraDebug)
	:  _radio(_cePin, _csPin), primary(primaryRadio), extraDebug(_extraDebug){ 

	//initialize encryption
	chacha.setNumRounds(cypher.rounds);
	chacha.setKey(cypher.key, cypher.keySize);
	chacha.setIV(cypher.iv, chacha.ivSize());		
}

void WOLF_COMS::begin() {
	_radio.begin();
	_radio.setPALevel(RF24_PA_MAX);
	_radio.setAutoAck(1);                    // Ensure autoACK is enabled
	_radio.enableAckPayload();               // Allow optional ack payloads
	_radio.setRetries(8, 10);                 // Smallest time between retries, max no. of retries
	
	if (primary) {
		_radio.openWritingPipe(pipes[0]);        // Both radios listen on the same pipes by default, and switch when writing
		_radio.openReadingPipe(1, pipes[1]);
	} else {
		_radio.openWritingPipe(pipes[1]);        // Both radios listen on the same pipes by default, and switch when writing
		_radio.openReadingPipe(1, pipes[0]);
	}	
	_radio.startListening();                 // Start listening
	_radio.printDetails();                   // Dump the configuration of the rf unit for debugging

	Serial.println("WOLF_COMS object initialization");
}

bool WOLF_COMS::sendMessage(nRF24Msg_union *_msg) {
	encryptMsg(_msg);
	_radio.stopListening();
	if (!_radio.write(&_msg->msg_bytes, sizeof(_msg->msg_bytes))) {
		Serial.println("Failed to send message");
		if (_radio.failureDetected) {
			Serial.print("radio hardware failure detected: ");
		}
		return false;
	}
	return true;
}

//TODO: return pipNo
bool WOLF_COMS::receiveMessage(nRF24Msg_union *_msg, byte *ackPipe) {
	if (_radio.available(ackPipe)) {
		_radio.read(_msg->msg_bytes, sizeof(_msg->msg_bytes));      // Get the payload
		decryptMsg(_msg);
		return true;
	}
	return false;
}

//store next ack payload to go out when a msg read occurs
//Flush TX buffer before calling this to avoid overflow
//TX buffer is only 3 messages deep
void WOLF_COMS::writeAckMessage(nRF24Msg_union *_msg, byte ackPipe) {		
	_radio.writeAckPayload(ackPipe, _msg->msg_bytes, sizeof(_msg->msg_bytes));	
}

bool WOLF_COMS::receiveAckMessage(nRF24Msg_union *_msg) {
	bool messageAvailable = false;
	while (_radio.isAckPayloadAvailable()) {
		messageAvailable = true;
		_radio.read(&_msg->msg_bytes, sizeof(_msg->msg_bytes));
		if (extraDebug) {
			Serial.print("Ack msg type is: ");
			Serial.println((char)_msg->nRF24Header_struct.msgType);
		}
	}
	return messageAvailable;
}

//Returns true is connection established, false if not
//_msg is message to send to check connection, use heartbeat
bool WOLF_COMS::establishConnectionTX(nRF24Msg_union *_msg) {
	if (!sendMessage(_msg)) {
		if (count < 20) {
			Serial.print(".");
			count++;
		} else {
			count = 0;
			Serial.println("!");
		}
		return false;
	}
	Serial.println(" ");
	Serial.println("Connection Established");

	return true;	
}

//establish a connection on a RX radio
//Call flushBuffer before calling establishConnectionRX
bool WOLF_COMS::establishConnectionRX() {		
	if (!_radio.available()) {		
		if (count < 20) {
			Serial.print(".");
			count++;
		} else {
			count = 0;
			Serial.println("!");
		}	
		return false;
	}

	Serial.println(" ");
	Serial.println("Connection Established");
	
	return true;
}

//flush rx buffer
void WOLF_COMS::flushRXBuffer() {	
	if (_radio.available()) {
		nRF24Msg_union buf;
		_radio.read(&buf.msg_bytes, sizeof(buf.msg_bytes));
		Serial.println("Flushing buffer");
	}
}

void WOLF_COMS::flushTXBuffer() {
	_radio.flush_tx();
}

void WOLF_COMS::encryptMsg(nRF24Msg_union *_msg) {
	randomSeed(millis());

	for (uint8_t i = 0; i < 8; i++) {
		_msg->controlMsg_struct.seed[i] = (byte)random(0, 255);
	}

	//print unencrypted message
	if (extraDebug) {
		printMsg(_msg);
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
		printMsg(_msg);
	}
}

void WOLF_COMS::decryptMsg(nRF24Msg_union * _msg) {
	//print unencrypted message
	if (extraDebug) {
		Serial.println("Encrypted message: ");
		printMsg(_msg);
	}

	byte seed[8];
	for (uint8_t i = 0; i < 8; i++) {
		seed[i] = _msg->msg_bytes[i];
	}

	chacha.setIV(cypher.iv, chacha.ivSize());
	if (!chacha.setCounter(seed, sizeof(seed))) {
		Serial.println("Failed to set decrypt counter!");
	}
	nRF24Msg_union decryptedMsg = {};
	chacha.decrypt(decryptedMsg.msg_bytes, _msg->msg_bytes, sizeof(_msg->msg_bytes));
	
	//print unencrypted message
	if (extraDebug) {
		Serial.println("Decrypted message: ");
		printMsg(_msg);
	}

	//copy decrypted data back into union
	memcpy(_msg->msg_bytes, decryptedMsg.msg_bytes, sizeof(decryptedMsg.msg_bytes));
}

void WOLF_COMS::printMsg(nRF24Msg_union *_msg) {
	for (uint8_t i = 0; i < sizeof(_msg->msg_bytes); i++) {
	Serial.print(_msg->msg_bytes[i]);
	Serial.print(", ");
	}
	Serial.println();
}

WOLF_COMS::~WOLF_COMS() {
}