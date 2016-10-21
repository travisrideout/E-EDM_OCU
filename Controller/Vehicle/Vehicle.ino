/*
 Name:		Vehicle.ino
 Created:	10/12/2016 16:21:08
 Author:	travis.rideout
*/

//TODO: keeping tring to re-establish connection when in fault

#include "Vehicle.h"

void setup() {
	Serial.begin(115200);
	printf_begin();

	//Print header
	Serial.println("HDT ROBOTICS");
	Serial.println("E-EDM VEHICLE");
	Serial.println("VERSION: 1.0");
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
	pinMode(ledRedPin, OUTPUT);
	pinMode(ledGreenPin, OUTPUT);
	pinMode(ledBluePin, OUTPUT);
	pinMode(clearFaultPin, INPUT_PULLUP);

	// Setup and configure rf radio
	radio.begin();
	radio.setPALevel(RF24_PA_MAX);
	//radio.setAutoAck(1);                    // Ensure autoACK is enabled
	//radio.enableAckPayload();               // Allow optional ack payloads
	radio.setRetries(0, 15);                 // Smallest time between retries, max no. of retries
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
			prev_time = millis();
			Serial.println("Tried to set fault");
			//setFault(Loss_Of_Signal);
		}
	} else {
		if (digitalRead(clearFaultPin) == 0) {
			unsigned long pressedTime = millis();
			bool clearButtonDown = true;
			while (clearButtonDown) {
				if (millis() - pressedTime > clearFaultTimer) {
					Serial.println("Fault Cleared");
					clearFault();
					clearButtonDown = false;
				} else {
					clearButtonDown = !digitalRead(clearFaultPin);
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

	led.red = 0;
	led.green = 0;
	led.blue = 0;

	setOutputs();
}

//checks for connection every 3sec
//double vibes while no connection
//long vibe once connection is made
bool establishConnection() {
	Serial.println("Waiting for connection");
	long startTime = millis();
	int count = 0;
	bool ledBlinkState = false;
	//flush rx buffer first
	while (radio.available()) {
		dataStruct buf;
		radio.read(&buf, sizeof(buf));
		Serial.println("Flushing buffer");
	}
	while (!radio.available()) {
		if (count < 20) {
			Serial.print(".");
			count++;
		} else {
			count = 0;
			Serial.println("!");
			
		}
		ledBlinkState = !ledBlinkState;
		if (ledBlinkState) {
			led.blue = 125;
		} else {
			led.blue = 0;
		}
		if (millis() - startTime > 60000) {
			Serial.println(" ");
			Serial.println("Unable to Establish a Connection");
			setFault(Unable_To_Establish_Connection);
			return false;
		}
		analogWrite(ledBluePin, led.blue);
		delay(1000);		
	}
	Serial.println(" ");
	Serial.println("Connection Established");
	led.blue = 0;
	led.green = 125;
	analogWrite(ledBluePin, led.blue);
	analogWrite(ledGreenPin, led.green);

	return true;
}

void parseMessage() {
	//joy axis to track speed/direction decoding
	//scale joystick commands outside deadband
	//out of range checks

	if (newValues.xAxis > 1023 || newValues.xAxis < 0 || newValues.yAxis > 1023 || newValues.yAxis < 0) {
		setFault(Input_Out_Of_Range);
		return;
	}

	if (newValues.deadman) {
		initializeData(newValues);
	}

	//map the joystick values to a -255 to 255 range outside of the deadband
	//TODO: get rid of deadband as controller is calibrated
	int mappedXAxis, mappedYAxis;
	if (newValues.xAxis > 512 + deadband) {
		mappedXAxis = map(newValues.xAxis, 512 + deadband, 1024, 0, 255);
	} else if (newValues.xAxis < 511 - deadband) {
		mappedXAxis = map(newValues.xAxis, 0, 512 - deadband, -255, 0);
	} else {
		mappedXAxis = 0;
	}
	if (newValues.yAxis > 512 + deadband) {
		mappedYAxis = map(newValues.yAxis, 512 + deadband, 1024, 0, 255);
	} else if (newValues.yAxis < 511 - deadband) {
		mappedYAxis = map(newValues.yAxis, 0, 512 - deadband, -255, 0);
	} else {
		mappedYAxis = 0;
	}

	//set the motor speeds and direction
	leftMotor.speed = constrain(mappedYAxis + mappedXAxis, -255, 255);
	rightMotor.speed = constrain(mappedYAxis - mappedXAxis, -255, 255);
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

	//buttons 1&2 not used yet

	setOutputs();
}

void setOutputs() {
	analogWrite(rightMotorSpeedPin, rightMotor.speed);
	analogWrite(rightMotorBrakePin, rightMotor.brake);
	if (rightMotor.direction) {
		digitalWrite(rightMotorReversePin, true);
		digitalWrite(rightMotorForwardPin, false);
	} else {
		digitalWrite(rightMotorReversePin, false);
		digitalWrite(rightMotorForwardPin, true);
	}
	digitalWrite(rightMotorInterlockPin, rightMotor.interlock);

	analogWrite(leftMotorSpeedPin, leftMotor.speed);
	analogWrite(leftMotorBrakePin, leftMotor.brake);
	if (leftMotor.direction) {
		digitalWrite(leftMotorReversePin, true);
		digitalWrite(leftMotorForwardPin, false);
	} else {
		digitalWrite(leftMotorReversePin, false);
		digitalWrite(leftMotorForwardPin, true);
	}
	digitalWrite(leftMotorInterlockPin, leftMotor.interlock);

	analogWrite(ledRedPin, led.red);
	analogWrite(ledGreenPin, led.green);
	analogWrite(ledBluePin, led.blue);
#ifdef debug
	printOutputs();
#endif // DEBUG
}

void printOutputs() {
	Serial.print(leftMotor.speed);
	Serial.print(", ");
	Serial.print(leftMotor.brake);
	Serial.print(", ");
	Serial.print(leftMotor.direction);
	Serial.print(", ");
	Serial.print(leftMotor.interlock);
	Serial.print(", ");
	Serial.print(rightMotor.speed);
	Serial.print(", ");
	Serial.print(rightMotor.brake);
	Serial.print(", ");
	Serial.print(rightMotor.direction);
	Serial.print(", ");
	Serial.print(rightMotor.interlock);
	Serial.print(", ");
	Serial.print(led.red);
	Serial.print(", ");
	Serial.print(led.green);
	Serial.print(", ");
	Serial.println(led.blue);
}

void setFault(faultCodes code) {
	fault_state = true;
	faultCode = code;
	initializeData(newValues);
	led.red = 125;
	led.green = 0;
	led.blue = 0;
	setOutputs();
}

void printFault() {
	Serial.print("Fault: ");
	Serial.println(faultCode);
}

void clearFault() {
	fault_state = false;
	faultCode = No_Fault;
	initializeData(newValues);
	led.red = 0;
	led.green = 0;
	led.blue = 0;
	setOutputs();
	establishConnection();
}
