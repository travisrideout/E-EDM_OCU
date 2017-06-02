/*
WOLF OCU RF Range tester
6/1/2017
*/

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Singleton instance of the radio driver
RH_RF95 driver(10, 8);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

void setup() {

	Serial.begin(115200);
	while (!Serial); // Wait for serial port to be available

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

unsigned long recieved_count = 0;
unsigned long missed_packets = 0;
bool first_message = true;

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void loop() {
	if (manager.available()) {
		uint8_t len = sizeof(data.bytes);
		uint8_t from;
		if (manager.recvfromAck(data.bytes, &len, &from)) {
			recieved_count++;

			//reset missed packet count on reset by adding offset
			if (first_message) {
				recieved_count = data.count;
				first_message = false;
				Serial.println("calibrated missed message counter");
			}
			
			missed_packets = data.count - recieved_count;

			Serial.print("got message from : 0x");
			Serial.print(from, HEX);
			Serial.print(": ");
			Serial.print(data.count);
			Serial.print(" RSSI: ");
			Serial.print(driver.lastRssi(), DEC);
			Serial.print(" Missed Packets: ");
			Serial.println(missed_packets);
			
			//Send a reply back to the originator client
			if (!manager.sendtoWait(data.bytes, sizeof(data.bytes), from)) {
				//Serial.println("sendtoWait failed");
			}				
		}
	}
}


