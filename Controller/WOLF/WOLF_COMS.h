#ifndef WOLF_COMS_h
#define WOLF_COMS_h

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Crypto.h>
#include <ChaCha.h>

class WOLF_COMS {
public:
	bool extraDebug = false;

	//message type id's
	static const char control = 'C';
	static const char fault = 'F';
	static const char heartbeat = 'H';
	static const char gui = 'G';
	static const char userFeedback = 'U';

	typedef struct _nRF24Header {
		byte seed[8];
		byte pairedID;
		byte msgType;
	}nRF24Header;

	typedef struct _controlMsg {
		byte seed[8];
		byte pairedID;
		byte msgType;
		int16_t xAxis;
		int16_t yAxis;
		byte button1;
		byte button2;
		byte deadman;
		byte ocuBatt;		//in percent of total 0-100
	}controlMsg;

	typedef struct _heartbeatMsg {
		byte seed[8];
		byte pairedID;
		byte msgType;
		unsigned long count;
	}heartbeatMsg;

	typedef struct _guiMsg {
		byte seed[8];
		byte pairedID;
		byte msgType;
		byte batteryLevel;
		byte fuelLevel;
		byte state;
		int16_t orient[3];
		byte mode;
		byte speed;
		unsigned int mileage;
		unsigned int hours;
		byte accessories;
	}guiMsg;

	typedef struct _faultMsg {
		byte seed[8];
		byte pairedID;
		byte msgType;
		int16_t faultCode;
		byte faultSpecificData[8];
	}faultMsg;

	typedef union _nRF24Msg_union {
		nRF24Header nRF24Header_struct;
		controlMsg controlMsg_struct;
		heartbeatMsg heartbeatMsg_struct;
		guiMsg guiMsg_struct;
		faultMsg faultMsg_struct;
		uint8_t msg_bytes[32];
	}nRF24Msg_union;	
	
	//Prototypes
	WOLF_COMS(uint8_t cePin, uint8_t csPin, bool primaryRadio = true, bool _extraDebug = false); 
	void begin();
	bool sendMessage(nRF24Msg_union *_msg);
	bool receiveMessage(nRF24Msg_union *_msg, byte *ackPipe);
	void writeAckMessage(nRF24Msg_union *_msg, byte ackPipe);
	bool receiveAckMessage(nRF24Msg_union *_msg);
	bool establishConnectionTX(nRF24Msg_union *_msg);
	bool establishConnectionRX();
	void flushRXBuffer();
	void flushTXBuffer();
	~WOLF_COMS();

private:
	bool primary = false;
	int count = 0;

	//Encryption objects
	 struct Encryption {
		byte key[32] = { 0x66, 0x74, 0x7a, 0xc2, 0x11, 0x9a, 0x36, 0x03,
			0x7e, 0x95, 0x85, 0x27, 0x1c, 0xf5, 0xfa, 0x4a,	0xb1,
			0x0d, 0xb6, 0xc5, 0x86, 0xe9, 0xcb, 0x53, 0x23, 0x60, 0x00,
			0x34, 0xae, 0x28, 0x81, 0xd7 }; 
		size_t keySize = 32;
		uint8_t rounds = 20;
		byte iv[8] = { 0x27, 0xe2, 0x9e, 0x10, 0xbd, 0x14, 0x2d, 0xba };
	};
	static const Encryption cypher;
	ChaCha chacha;

	RF24 _radio;
	const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };

	//Prototypes
	void printMsg(nRF24Msg_union *_msg);
	void encryptMsg(nRF24Msg_union *_msg);
	void decryptMsg(nRF24Msg_union *_msg);

};

#endif //WOLF_COMS.h