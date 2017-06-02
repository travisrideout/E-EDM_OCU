/* 
WOLF OCU RF Range tester
6/1/2017
*/

#include <RHReliableDatagram.h>
#include <RH_RF95.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

#define RFM95_CS 27
#define RFM95_RST 30
#define RFM95_INT 11
#define RED_LED		15
#define GREEN_LED	16

// Singleton instance of the radio driver
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

void setup() {
	pinMode(RFM95_RST, INPUT);	//rst is an output on rfm95,
	pinMode(RED_LED, OUTPUT);
	pinMode(GREEN_LED, OUTPUT);

	digitalWrite(RED_LED, HIGH);
	digitalWrite(GREEN_LED, LOW);

	Serial.begin(115200);

	//Manual reset of radio
	Serial.println("Reset Radio");
	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, LOW);
	delayMicroseconds(100);
	pinMode(RFM95_RST, INPUT);
	delay(10);

	const uint8_t retryTimeout = 25;		//time between retries in ms	
	const uint8_t retries = 2;

	if (!manager.init()) {
		Serial.println("init failed");
	} else {
		Serial.println("init passed");
	}
	manager.setTimeout(retryTimeout);
	manager.setRetries(retries);

	if (!driver.setFrequency(915.0)) {
		Serial.println("setFrequency failed");
	} else {
		Serial.println("Frequency set to 915.0 MHz");
	}

	driver.setTxPower(23, false);
	driver.setModemConfig(driver.Bw500Cr45Sf128);
}

union merge {
	unsigned long count = 0;
	uint8_t bytes[4];
};

merge data;

merge ack_data;

void loop() {
	data.count++;

	if (manager.sendtoWait(data.bytes, sizeof(data.bytes), SERVER_ADDRESS)) {
		digitalWrite(RED_LED, LOW);
		digitalWrite(GREEN_LED, HIGH);
		
		Serial.print("Sending message #: ");
		Serial.println(data.count);

		uint8_t len = sizeof(ack_data.bytes);
		uint8_t from;
		if (manager.recvfromAckTimeout(ack_data.bytes, &len, 200, &from)) {
			digitalWrite(RED_LED, LOW);
			digitalWrite(GREEN_LED, HIGH);
			
			Serial.print("got reply from : 0x");
			Serial.print(from, HEX);
			Serial.print(": ");
			Serial.print(ack_data.count);
			Serial.print(" RSSI: ");
			Serial.println(driver.lastRssi(), DEC);
		} else {
			digitalWrite(RED_LED, HIGH);
			digitalWrite(GREEN_LED, LOW);
			Serial.println("No reply, is rf95_reliable_datagram_server running?");
		}
	} else {
		digitalWrite(RED_LED, HIGH);
		digitalWrite(GREEN_LED, LOW);
		Serial.println("sendtoWait failed");
	}		
	delay(100);
}


