/*
 Name:		Controller.ino
 Created:	10/12/2016 16:19:26
 Author:	travis.rideout
*/

//TODO: do heartbeat on interrupt timer
//TODO: utilize low power modes to save battery

#include "Controller.h"

int count = 0;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	printf_begin();

	//Print header
	Serial.println("HDT ROBOTICS");
	Serial.println("E-EDM CONTROLLER");
	Serial.println("VERSION: 1.0");
	Serial.println(" ");

	//Set pin modes
	pinMode(xAxisPin, INPUT);
	pinMode(yAxisPin, INPUT);
	pinMode(button1Pin, INPUT_PULLUP);	//low, 0, is pressed. High, 1, is not pressed
	pinMode(button2Pin, INPUT_PULLUP);	//low, 0, is pressed. High, 1, is not pressed
	pinMode(deadmanPin, INPUT_PULLUP);	//low, 0, is pressed. High, 1, is not pressed
	pinMode(vibePin, OUTPUT);	

	// Setup and configure rf radio
	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	//radio.setAutoAck(1);                    // Ensure autoACK is enabled
	//radio.enableAckPayload();               // Allow optional ack payloads
	radio.setRetries(0, 15);                 // Smallest time between retries, max no. of retries
	radio.openWritingPipe(pipes[0]);        // Both radios listen on the same pipes by default, and switch when writing
	radio.openReadingPipe(1, pipes[1]);
	radio.startListening();                 // Start listening
	radio.printDetails();                   // Dump the configuration of the rf unit for debugging

	//initialize data
	initializeData(oldValues);
	initializeData(newValues);
	initializeData(heartbeat);
	initializeJoysticks();

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
		prev_time = millis();
	} else if (millis() - prev_time > heartbeat_timeout) {
		Serial.print("sending heartbeat message: ");
		Serial.println(count);
		count++;
		if (!sendData(heartbeat)) {
			establishConnection();
		}
		prev_time = millis();
	}	
	calibrateJoystick();
}

//initialize data struct to zero values
void initializeData(dataStruct &data) {
	data.xAxis = 512;
	data.yAxis = 512;
	data.button1 = true;
	data.button2 = true;
	data.deadman = true;
}

void initializeJoysticks() {
	EEPROM.get(xJoyMemLoc, xAxis);
	EEPROM.get(yJoyMemLoc, yAxis);
	Serial.println("Joystick Calibration Pulled from Memory:");
	printJoystickCalibration();
}

void printJoystickCalibration() {
	Serial.print("-X Axis- Center: ");
	Serial.print(xAxis.center);
	Serial.print(", Min: ");
	Serial.print(xAxis.min);
	Serial.print(", Max: ");
	Serial.print(xAxis.max);
	Serial.print(", Deadband: ");
	Serial.println(xAxis.deadband);
	Serial.print("-Y Axis- Center: ");
	Serial.print(yAxis.center);
	Serial.print(", Min: ");
	Serial.print(yAxis.min);
	Serial.print(", Max: ");
	Serial.print(yAxis.max);
	Serial.print(", Deadband: ");
	Serial.println(yAxis.deadband);
}

//if deadman is pressed read values, else reset all values
bool getInputs() {
	if (!digitalRead(deadmanPin)) {
		//shuffle values back into history
		shuffleValues(oldValues, newValues);
		//read new values
		newValues.deadman = digitalRead(deadmanPin);
		newValues.button1 = digitalRead(button1Pin);
		newValues.button2 = digitalRead(button2Pin);
		newValues.xAxis = scaleAxis(xAxis, analogRead(xAxisPin));
		newValues.yAxis = scaleAxis(yAxis, analogRead(yAxisPin));
#ifdef debug
		printValues();
#endif // DEBUG
		return true;
	} else {
		shuffleValues(oldValues, newValues);
		initializeData(newValues);
		return false;
	}
}

//Scale a joystick axis reading between its min/max around its center +/- deadband
int scaleAxis(joystick joy, int val) {
	if (val > joy.center + joy.deadband) {
		if (val > joy.max) {
			return 1023;
		} else {			
			return (int)map(val, joy.center + joy.deadband, joy.max, 513, 1023);
		}
	} else if (val < joy.center - joy.deadband) {
		if (val < joy.min) {
			return 0;
		} else {			
			return (int) map(val, joy.min, joy.center - joy.deadband, 0, 511);
		}
	} else {
		return 512;
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
		vibe(1,100);
		delay(3000);		
	}
	Serial.println(" ");
	Serial.println("Connection Established");
	vibe(2, 100, 100);
	
	return true;
}

//Check that the joystick is within centered range and no buttons are shorted
//Allow joystick calibration
bool selfCheck() {
	Serial.println("Checking for Controller Faults");
	Serial.println("RELEASE ALL CONTROLLER BUTTONS!");
	int count = 0;
	Serial.print(analogRead(xAxisPin));
	Serial.print(", ");
	Serial.print(analogRead(yAxisPin));
	Serial.print(", ");
	Serial.print(digitalRead(deadmanPin));
	Serial.print(", ");
	Serial.print(digitalRead(button1Pin));
	Serial.print(", ");
	Serial.println(digitalRead(button2Pin));
	if (digitalRead(deadmanPin) == 1 && digitalRead(button1Pin) == 1 && digitalRead(button2Pin) == 1 &&
		analogRead(xAxisPin) < xAxis.center + xAxis.deadband &&
		analogRead(xAxisPin) > xAxis.center - xAxis.deadband &&
		analogRead(yAxisPin) < yAxis.center + yAxis.deadband &&
		analogRead(yAxisPin) > yAxis.center - yAxis.deadband) {
		Serial.println("Controller checks OK");		
		vibe(1, 100, 1000);
		return true;
	} else {
		//Stick here until reset		
		while (1) {
			Serial.println("Controller Failed Self-check");
			Serial.println("Restart or Try Calibrating Joystick");
			vibe(3, 200, 200);
			if (calibrateJoystick()) {
				if (selfCheck()) {
					return true;
				}
			}
			delay(3000);
		}
	}	
}

//Two step process to get center/min/max values of joystick range
bool calibrateJoystick() {
	if (digitalRead(deadmanPin) && !digitalRead(button1Pin) && !digitalRead(button2Pin)) {
		if (!calibrate) {
			calibrate = true;
			calibrate_time = millis();
			return false;
		}
	} else {
		calibrate = false;
		return false;
	}
	if (calibrate && millis() - calibrate_time > calibrate_timeout) {
		Serial.println("JOYSTICK CALIBRATION STARTED");
		Serial.println("Leave Joystick Centered");
		vibe(1, 500);
		xAxis.center = analogRead(xAxisPin);
		yAxis.center = analogRead(yAxisPin);
		xAxis.deadband = 20;
		yAxis.deadband = 20;
		delay(2000);
		Serial.println("Circle Joystick");
		vibe(2, 500, 500);
		int time = millis();
		xAxis.min = xAxis.center;
		xAxis.max = xAxis.center;
		yAxis.min = yAxis.center;
		yAxis.max = yAxis.center;
		while (millis() - time < 5000) {
			int xpos = analogRead(xAxisPin);
			int ypos = analogRead(yAxisPin);
			if (xpos < xAxis.min) {
				xAxis.min = xpos;
			} else if (xpos > xAxis.max) {
				xAxis.max = xpos;
			}
			if (ypos < yAxis.min) {
				yAxis.min = ypos;
			} else if (ypos > yAxis.max) {
				yAxis.max = ypos;
			}
			printJoystickCalibration();
		}
		EEPROM.put(xJoyMemLoc, xAxis);
		EEPROM.put(yJoyMemLoc, yAxis);
		vibe(3, 500, 500);
		Serial.println();
		Serial.println("JOYSTICK CALIBRATION COMPLETE");
		printJoystickCalibration();
		calibrate = false;
		return true;
	}
	return false;
}

void printValues() {
	Serial.print(newValues.xAxis);
	Serial.print(", ");
	Serial.print(newValues.yAxis);
	Serial.print(", ");
	Serial.print(newValues.button1);
	Serial.print(", ");
	Serial.print(newValues.button2);
	Serial.print(", ");
	Serial.println(newValues.deadman);
}
