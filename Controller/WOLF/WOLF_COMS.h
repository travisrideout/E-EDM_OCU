#ifndef WOLF_COMS_h
#define WOLF_COMS_h

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Crypto.h>
#include <ChaCha.h>

class WOLF_COMS {
public:
	bool extraDebug = false;
	
	//CAN message type id's
	static const char control = 'C';	
	static const char heartbeat = 'H';
	static const char gui = 'G';
	static const char fault = 'F';
	static const char userFeedback = 'U';
	static const char imu = 'I';

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
		char ocuRSSI;
	}controlMsg;

	typedef struct _heartbeatMsg {
		byte seed[8];
		byte pairedID;
		byte msgType;
		uint32_t count;
	}heartbeatMsg;

	typedef struct _guiMsg {
		byte seed[8];
		byte pairedID;
		byte msgType;
		byte batteryLevel;
		byte fuelLevel;
		byte state;
		int16_t orient[3];	//[0] = y, [1] = z, [2] = x
		byte imuStatus;
		byte mode;
		byte speed;
		uint32_t mileage;
		uint32_t hours;
		byte accessories;
		char vehicleRSSI;
	}guiMsg;

	typedef struct _faultMsg {
		byte seed[8];
		byte pairedID;
		byte msgType;
		uint16_t faultCode;
		byte faultSpecificData[6];
	}faultMsg;

	typedef struct _userFeedbackMsg {
		byte seed[8];
		byte pairedID;
		byte msgType;
		byte modeReq;
		byte request;
		byte accessories;
	}userFeedbackMsg;

	typedef union _nRF24Msg_union {
		nRF24Header nRF24Header_struct;
		controlMsg controlMsg_struct;
		heartbeatMsg heartbeatMsg_struct;
		guiMsg guiMsg_struct;
		faultMsg faultMsg_struct;
		userFeedbackMsg userFeedbackMsg_struct;
		uint8_t msg_bytes[32];
	}RFMsg_union;	
	
	//Prototypes
	WOLF_COMS(uint8_t csPin, uint8_t intPin, uint8_t resetPin, uint8_t address, bool _extraDebug = false);
	void begin();
	bool sendMessage(RFMsg_union *_msg, uint8_t address);
	bool receiveMessage(RFMsg_union *_msg, byte *ackAddress);
	bool receiveAckMessage(RFMsg_union *_msg, byte *ackAddress);
	void printMessage(RFMsg_union *msg);
	~WOLF_COMS();

private:
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

	RH_RF95 driver;
	RHReliableDatagram manager;
	const double RF95_FREQ = 915.0;
	uint8_t rstPin;
	const uint8_t retryTimeout = 25;		//time between retries in ms	
	const uint8_t retries = 2;			//number of retries before failure
	const uint8_t ackTimeout = retryTimeout * retries + 5;		//time to wait for ack response in ms

	//Prototypes
	void encryptMsg(RFMsg_union *_msg);
	void decryptMsg(RFMsg_union *_msg);
};

#endif //WOLF_COMS.h