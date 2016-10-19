/*
 Name:		Controller.ino
 Created:	10/12/2016 16:19:26
 Author:	travis.rideout
*/

#include "Controller.h"

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);

	//Print header
	Serial.println("HDT ROBOTICS");
	Serial.println("E-EDM CONTROLLER");
	Serial.println("VERSION: 0.01 ALPHA");
	Serial.println(" ");

	//Set pin modes
	pinMode(xAxisPin, INPUT);
	pinMode(yAxisPin, INPUT);
	pinMode(button1Pin, INPUT);
	pinMode(button2Pin, INPUT);
	pinMode(deadmanPin, INPUT);
	pinMode(vibePin, OUTPUT);	

	// Setup and configure rf radio
	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);
	radio.setAutoAck(1);                    // Ensure autoACK is enabled
	radio.enableAckPayload();               // Allow optional ack payloads
	radio.setRetries(10, 15);                 // Smallest time between retries, max no. of retries
	radio.setPayloadSize(sizeof(newValues));                // Here we are sending 1-byte payloads to test the call-response speed
	radio.openWritingPipe(pipes[1]);        // Both radios listen on the same pipes by default, and switch when writing
	radio.openReadingPipe(1, pipes[0]);
	radio.startListening();                 // Start listening
	radio.printDetails();                   // Dump the configuration of the rf unit for debugging

	//initialize data
	initializeData(oldValues);
	initializeData(newValues);
	initializeData(heartbeat);

	prev_time = millis();

	selfCheck();
	establishConnection();	
}

// the loop function runs over and over again until power down or reset
void loop() {	
	if (millis() - prev_time > message_frequency && getInputs()) {
		if (!sendData(newValues)) {
			establishConnection();
		}
	} else if (millis() - prev_time > heartbeat_timeout) {
		if (!sendData(heartbeat)) {
			establishConnection();
		}
		prev_time = millis();
	}
}

//initialize data struct to zero values
void initializeData(dataStruct &data) {
	data.xAxis = 512;
	data.yAxis = 512;
	data.button1 = false;
	data.button2 = false;
	data.deadman = false;
}

//if deadman is pressed read values, else reset all values
bool getInputs() {
	if (digitalRead(deadmanPin)) {
		//shuffle values back into history
		shuffleValues(oldValues, newValues);
		//read new values
		newValues.deadman = digitalRead(deadmanPin);
		newValues.button1 = digitalRead(button1Pin);
		newValues.button2 = digitalRead(button2Pin);
		newValues.xAxis = analogRead(xAxisPin);
		newValues.yAxis = analogRead(yAxisPin);	
#ifdef DEBUG
		printValues();
#endif // DEBUG
		return true;
	} else {
		shuffleValues(oldValues, newValues);
		initializeData(newValues);
		return false;
	}
}

void shuffleValues(dataStruct &oldData, dataStruct &newData) {
	oldData.deadman = newData.deadman;
	oldData.button1 = newData.button1;
	oldData.button2 = newData.button2;
	oldData.xAxis = newData.xAxis;
	oldData.yAxis = newData.yAxis;
}

bool compareValues(dataStruct &oldData, dataStruct &newData) {
	if (oldData.button1 != newData.button1 || oldData.button2 != newData.button2
		|| oldData.xAxis != newData.xAxis || oldData.yAxis != newData.yAxis) {
		return true;
	} else {
		return false;
	}
}

bool sendData(dataStruct &data) {
	radio.stopListening();                                  // First, stop listening so we can talk.
	if (!radio.write(&data, sizeof(data))) {
		Serial.println("Failed to send message");
		return false;
	} 
	//implement ack messages here
	/*else {
		if (!radio.available()) {
			Serial.println(F("Blank Payload Received."));
		} else {
			while (radio.available()) {
				unsigned long tim = micros();
				radio.read(&gotByte, 1);
			}
		}*/	
	return true;
}

//vibe the controller
//BLOCKS processor
void vibe(int numPulses = 1, int pulseDuration = 100, int pulseDelay = 100) {
	for (int i = 0; i < numPulses; i++) {
		digitalWrite(vibePin, 1);
		delay(pulseDuration);
		digitalWrite(vibePin, 0);
		delay(pulseDelay);
	}
}

//checks for connection every 3sec
//double vibes while no connection
//long vibe once connection is made
bool establishConnection() {
	Serial.println("Waiting for connection");
	int count = 0;
	while (!sendData(heartbeat)) {
		if (count < 20) {
			Serial.print(".");
			count++;
		} else {
			count = 0;
			Serial.println("!");
		}
		vibe(2);
		delay(3000);		
	}
	Serial.println(" ");
	Serial.println("Connection Established");
	vibe(1, 1000, 0);
	
	return true;
}

void selfCheck() {
	getInputs();
	Serial.println("Checking for Controller Faults");
	Serial.println("RELEASE ALL CONTROLLER BUTTONS!");
	int count = 0;
	while (digitalRead(deadmanPin) != false && 
		digitalRead(button1Pin) != false && 
		digitalRead(button2Pin) != false &&
		analogRead(xAxisPin) < 512 - deadband &&
		analogRead(xAxisPin) > 512 + deadband &&
		analogRead(yAxisPin) < 512 - deadband &&
		analogRead(yAxisPin) > 512 + deadband) {
		if (count < 20) {
			Serial.print(".");
			count++;
		} else {
			count = 0;
			Serial.println("!");
		}
		vibe(1, 100, 1000);
	}
	Serial.println("Controller checks OK");
}

void printValues() {
	Serial.print(newValues.xAxis, 3);
	Serial.print(", ");
	Serial.print(newValues.yAxis, 3);
	Serial.print(", ");
	Serial.print(newValues.button1, 1);
	Serial.print(", ");
	Serial.print(newValues.button2, 1);
	Serial.print(", ");
	Serial.println(newValues.deadman, 1);
}
