/*
 Name:		Vehicle.ino
 Created:	10/12/2016 16:21:08
 Author:	travis.rideout
*/

#include "Vehicle.h"

void setup() {
	Serial.begin(115200);

	//Print header
	Serial.println("HDT ROBOTICS");
	Serial.println("E-EDM VEHICLE");
	Serial.println("VERSION: 0.01 ALPHA");
	Serial.println(" ");

	//Set pin modes
	pinMode(rightMotorSpeedPin, OUTPUT);
	pinMode(rightMotorBrakePin, OUTPUT);
	pinMode(rightMotorForwardPin, OUTPUT);
	pinMode(rightMotorReversePin, OUTPUT);
	pinMode(rightMotorInterlockPin, OUTPUT);
	pinMode(leftMotorSpeedPin, OUTPUT);
	pinMode(leftMotorBrakePin, OUTPUT);
	pinMode(leftMotorForwardPin, OUTPUT);
	pinMode(leftMotorReversePin, OUTPUT);
	pinMode(leftMotorInterlockPin, OUTPUT);
	pinMode(clearFaultPin, INPUT_PULLUP);

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
	initializeData(newValues);
	initializeOutputs();

	prev_time = millis();
	establishConnection();
}

// the loop function runs over and over again until power down or reset
void loop() {
	//read message
	//parse message into outputs
	//if no message within heartbeat timeout then set outputs to zero
	if (!fault_state) {
		if (radio.available()) {
			while (radio.available()) {                          // While there is data ready
				radio.read(&newValues, sizeof(newValues));             // Get the payload
			}
			parseMessage();
			prev_time = millis();
		} else if (millis() - prev_time > heartbeat_timeout) {
			setFault(Loss_Of_Signal);
		}
	} else {
		if (digitalRead(clearFaultPin) == 0) {
			unsigned long pressedTime = millis();
			while (digitalRead(clearFaultPin) == 0) {
				if (millis() - pressedTime > clearFaultTimer) {
					Serial.println("Fault Cleared");
					clearFault();
				}
				delay(50);	//debounce
			}
		}
		if (millis() - prev_time > faultPrintTimer) {
			printFault();
			prev_time = millis();
		}
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

//initialize output object values
//TODO: interlock on/off?
void initializeOutputs() {
	leftMotor.speed = 0;
	leftMotor.brake = 0;
	leftMotor.direction = 1;
	leftMotor.interlock = 0;

	rightMotor.speed = 0;
	rightMotor.brake = 0;
	rightMotor.direction = 1;
	rightMotor.interlock = 0;

	setOutputs();
}

//checks for connection every 3sec
//double vibes while no connection
//long vibe once connection is made
bool establishConnection() {
	Serial.println("Waiting for connection");
	int count = 0;
	while (!radio.available()) {
		if (count < 20) {
			Serial.print(".");
			count++;
		} else {
			count = 0;
			Serial.println("!");
		}
		delay(1000);
	}
	Serial.println(" ");
	Serial.println("Connection Established");

	return true;
}

void parseMessage() {
	//joy axis to track speed/direction decoding
	//scale joystick commands outside deadband
	//out of range checks

	if (newValues.xAxis > 1024 || newValues.xAxis < 0 || newValues.yAxis > 1024 || newValues.yAxis < 0) {
		setFault(Input_Out_Of_Range);
		return;
	}

	if (!newValues.deadman) {
		initializeData(newValues);
	}

	//map the joystick values to a -255 to 255 range outside of the deadband
	int mappedXAxis, mappedYAxis;
	if (newValues.xAxis > 512 + deadband) {
		mappedXAxis = map(newValues.xAxis, 512 + deadband, 1024, 0, 255);
	} else if (newValues.xAxis < 512 - deadband) {
		mappedXAxis = map(newValues.xAxis, 0, 512 - deadband, -255, 0);
	} else {
		mappedXAxis = 0;
	}
	if (newValues.yAxis > 512 + deadband) {
		mappedYAxis = map(newValues.yAxis, 512 + deadband, 1024, 0, 255);
	} else if (newValues.yAxis < 512 - deadband) {
		mappedYAxis = map(newValues.yAxis, 0, 512 - deadband, -255, 0);
	} else {
		mappedYAxis = 0;
	}

	//set the motor speeds and direction
	leftMotor.speed = constrain(mappedXAxis + mappedYAxis, -255, 255);
	rightMotor.speed = constrain(mappedXAxis - mappedYAxis, -255, 255);
	if (leftMotor.speed > 0) {
		leftMotor.direction = 0;
	} else {
		leftMotor.direction = 1;
		leftMotor.speed = abs(leftMotor.speed);
	}
	if (rightMotor.speed > 0) {
		rightMotor.direction = 0;
	} else {
		rightMotor.direction = 1;
		rightMotor.speed = abs(rightMotor.speed);
	}

	setOutputs();
}

void setOutputs() {
	analogWrite(rightMotorSpeedPin, constrain(map(rightMotor.speed, 0, 100, 0, 255), 0, 255));
	analogWrite(rightMotorBrakePin, constrain(map(rightMotor.brake, 0, 100, 0, 255), 0, 255));
	if (rightMotor.direction) {
		digitalWrite(rightMotorReversePin, true);
		digitalWrite(rightMotorForwardPin, false);
	} else {
		digitalWrite(rightMotorReversePin, false);
		digitalWrite(rightMotorForwardPin, true);
	}
	digitalWrite(rightMotorInterlockPin, rightMotor.interlock);

	analogWrite(leftMotorSpeedPin, constrain(map(leftMotor.speed, 0, 100, 0, 255), 0, 255));
	analogWrite(leftMotorBrakePin, constrain(map(leftMotor.brake, 0, 100, 0, 255), 0, 255));
	if (leftMotor.direction) {
		digitalWrite(leftMotorReversePin, true);
		digitalWrite(leftMotorForwardPin, false);
	} else {
		digitalWrite(leftMotorReversePin, false);
		digitalWrite(leftMotorForwardPin, true);
	}
	digitalWrite(leftMotorInterlockPin, leftMotor.interlock);

}

void setFault(faultCodes code) {
	fault_state = true;
	faultCode = code;
	initializeData(newValues);
	setOutputs();
}

void printFault() {
	Serial.print("Fault: ");
	Serial.println(faultCode);
}

void clearFault() {
	fault_state = false;
	faultCode = No_Fault;
}