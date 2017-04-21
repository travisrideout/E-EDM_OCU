/*
 Name:		Feather_Controller.ino
 Created:	4/5/2017 09:57:14
 Author:	Travis.rideout
*/

//TODO: add BLE for operator GUI
//TODO: Implement rx from vehicle
//TODO: printf not working
//TODO: create error handler

//TODO: utilize sleep modes on no user interaction timeouts to save battery
//TODO: Programatically set radio PAlevel based on signal strength/lossed packets
//TODO: frequency hopping
//TODO: Frequency scan on startup
//TODO: measure battery voltage
//TODO: Implement controller ID's to avoid multiple controller interference
//TODO: Disallow deadman initialization with joystick not centered or buttons pressed

#include "Feather_Controller.h"

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
	printf_begin();	// doesn't work on nrf52

	// Bluefruit module must be initialized for Nffs to work
	// Since Bluefruit's SOC event handling task is required for flash operation
	Bluefruit.begin();

	// Initialize Nffs
	Nffs.begin();

	//Print header
	Serial.println("HDT ROBOTICS");
	Serial.println("E-EDM FEATHER CONTROLLER");
	Serial.println("VERSION: 0.1");
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
	radio.setAutoAck(1);                    // Ensure autoACK is enabled
	radio.enableAckPayload();               // Allow optional ack payloads
	radio.setRetries(8, 10);                 // Smallest time between retries, max no. of retries
	radio.openWritingPipe(pipes[0]);        // Both radios listen on the same pipes by default, and switch when writing
	radio.openReadingPipe(1, pipes[1]);
	radio.startListening();                 // Start listening
	radio.printDetails();                   // Dump the configuration of the rf unit for debugging
	Serial.println("Init Radio");

	//initialize data	
	initializeData(newValues);
	initializeJoysticks();

	//initialize encryption
	chacha.setNumRounds(cypher.rounds);
	chacha.setKey(cypher.key, cypher.keySize);
	chacha.setIV(cypher.iv, chacha.ivSize());
		
	Scheduler.startLoop(vibeTask);

	prev_time = millis();

	//selfCheck();
	establishConnection();
	getBatt();	
}

//realtime loop, not delayed
void loop() {		
	if (ocuCalibrateRequest()) {
		calibrateJoystick();
	}	

	//All nRF24 calls should be from here to avoid overlap
	if (millis() - prev_time > message_frequency && getInputs()) {
		prev_time = millis();
		nRF24Msg_union send = {};
		send.controlMsg_struct = newValues;
		encryptMsg(&send);
		if (!sendData(&send)) {
			establishConnection();
		}
	} else if (millis() - prev_time > heartbeat_frequency) {
		prev_time = millis();
		Serial.print("sending heartbeat message: ");
		count++;
		Serial.println(count);		
		nRF24Msg_union send = {};
		send.heartbeatMsg_struct = heartbeat;
		send.heartbeatMsg_struct.count = count;
		send.heartbeatMsg_struct.msgType = 'H';
		encryptMsg(&send);
		if (!sendData(&send)) {
			establishConnection();
		} 		
	}
	//read ack messages
	//TODO: sort and parse messages, shuttle to BT
	while (radio.isAckPayloadAvailable()) {
		nRF24Msg_union ackMsg = {};
		radio.read(&ackMsg.msg_bytes, sizeof(ackMsg.msg_bytes));
		Serial.print("Ack msg type is: ");
		Serial.println((char)ackMsg.nRF24Header_struct.msgType);
		/*Serial.print("Ack msg = ");
		for (uint8_t i = 0; i < sizeof(ackMsg.msg_bytes); i++) {
			Serial.print(ackMsg.msg_bytes[i]);
			Serial.print(", ");
		}
		Serial.println();*/

		Serial.print("pitch = ");
		Serial.print((float)ackMsg.guiMsg_struct.orient[0]/100.0f);
		Serial.print(", roll = ");
		Serial.print((float)ackMsg.guiMsg_struct.orient[1]/100.0f);
		Serial.print(", heading = ");
		Serial.println((float)ackMsg.guiMsg_struct.orient[2] / 100.0f);
	}
}

//loop for vibe motor, runs on 100ms delays
void vibeTask() {
	if (vibePulses) {
		if (vibeDurationCounter) {
			if (vibeDurationCounter == vibeDuration) {
				digitalWrite(vibePin, HIGH);
			}
			vibeDurationCounter--;
		} else if (vibeDelayCounter) {
			if (vibeDelayCounter == vibeDelay) {
				digitalWrite(vibePin, LOW);
			}
			vibeDelayCounter--;
		} else {
			vibePulses--;
			vibeDurationCounter = vibeDuration;
			vibeDelayCounter = vibeDelay;
		}
	}
	delay(vibe_frequency);
}

//Set a vibe command. Runs on interrupt timer1. All times in multiples of 100 ms
void setVibe(int numPulses = 1, int pulseDuration = 1, int pulseDelay = 1) {
	vibePulses = numPulses;
	vibeDuration = pulseDuration;
	vibeDurationCounter = pulseDuration;
	vibeDelay = pulseDelay;
	vibeDelayCounter = pulseDelay;
}

//initialize data struct to zero values
void initializeData(controlMsg &data) {
	data.pairedID = 201;
	data.msgType = 'C';
	data.xAxis = 512;
	data.yAxis = 512;
	data.button1 = true;
	data.button2 = true;
	data.deadman = true;
	data.ocuBatt = 100;
}

//open config file, error if not found -> force calibrate, create config file there
//Pull values from config file, validity check, initialize working variables
void initializeJoysticks() {	
	file.open(FILENAME, FS_ACCESS_READ);
	if (file.exists()) {
		Serial.println(FILENAME " file opened");

		//create byte unions of joystick structs to write to
		joyConfig_union x_union = {};
		joyConfig_union y_union = {};

		//read from file and store in unions
		file.read(x_union.joyConfig_bytes, sizeof(x_union.joyConfig_bytes));
		file.read(y_union.joyConfig_bytes, sizeof(y_union.joyConfig_bytes));
				
		//validity check values and set 
		if (x_union.joyConfig_struct.center > 0 && x_union.joyConfig_struct.center < 1023) {
			xAxis.center = x_union.joyConfig_struct.center;
		} else {
			//error
		}
		if (x_union.joyConfig_struct.min > 0 && x_union.joyConfig_struct.min < 1023) {
			xAxis.min = x_union.joyConfig_struct.min;
		} else {
			//error
		}
		if (x_union.joyConfig_struct.max > 0 && x_union.joyConfig_struct.max < 1023) {
			xAxis.max = x_union.joyConfig_struct.max;
		} else {
			//error
		}
		if (x_union.joyConfig_struct.deadband > 0 && x_union.joyConfig_struct.deadband < 1023) {
			xAxis.deadband = x_union.joyConfig_struct.deadband;
		} else {
			//error
		}
		if (y_union.joyConfig_struct.center > 0 && y_union.joyConfig_struct.center < 1023) {
			yAxis.center = y_union.joyConfig_struct.center;
		} else {
			//error
		}
		if (y_union.joyConfig_struct.min > 0 && y_union.joyConfig_struct.min < 1023) {
			yAxis.min = y_union.joyConfig_struct.min;
		} else {
			//error
		}
		if (y_union.joyConfig_struct.max > 0 && y_union.joyConfig_struct.max < 1023) {
			yAxis.max = y_union.joyConfig_struct.max;
		} else {
			//error
		}
		if (y_union.joyConfig_struct.deadband > 0 && y_union.joyConfig_struct.deadband < 1023) {
			yAxis.deadband = y_union.joyConfig_struct.deadband;
		} else {
			//error
		}
	} else  {
		//no config file was found so creating a new one with default values
		Serial.print("Open " FILENAME " file to write ... ");

		if (file.open(FILENAME, FS_ACCESS_WRITE)) {
			Serial.println("OK");
			joyConfig_union x_union = { joyCenterDefault,joyMinDefault,joyMaxDefault,joyDeadbandDefault };
			xAxis = x_union.joyConfig_struct;
			joyConfig_union y_union = { joyCenterDefault,joyMinDefault,joyMaxDefault,joyDeadbandDefault };
			yAxis = y_union.joyConfig_struct;
			file.write(x_union.joyConfig_bytes, sizeof(x_union.joyConfig_bytes));
			file.write(y_union.joyConfig_bytes, sizeof(y_union.joyConfig_bytes));
			file.close();			
		} else {
			//failed to create file
			Serial.println("Failed (hint: path must start with '/') ");
			Serial.print("errnum = ");
			Serial.println(file.errnum);
		}
	}	
	printJoystickCalibration();
}

//Two step process to get center/min/max values of joystick range
void calibrateJoystick() {
	Serial.println("JOYSTICK CALIBRATION STARTED");
	Serial.println("Leave Joystick Centered");
	setVibe(1, 5);
	xAxis.center = analogRead(xAxisPin);
	yAxis.center = analogRead(yAxisPin);
	xAxis.deadband = joyDeadbandDefault;
	yAxis.deadband = joyDeadbandDefault;
	delay(2000);
	Serial.println("Circle Joystick");
	setVibe(2, 5, 5);
	int time = millis();
	xAxis.min = xAxis.center;
	xAxis.max = xAxis.center;
	yAxis.min = yAxis.center;
	yAxis.max = yAxis.center;
	int xpos, ypos;
	while (millis() - time < 5000) {
		xpos = analogRead(xAxisPin);
		ypos = analogRead(yAxisPin);
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

	Serial.print("Open " FILENAME " file to write ... ");

	if (file.open(FILENAME, FS_ACCESS_WRITE)) {
		Serial.println("OK");
		//create byte unions of joystick structs to write to
		joyConfig_union x_union = {};
		joyConfig_union y_union = {};

		x_union.joyConfig_struct = xAxis;
		y_union.joyConfig_struct = yAxis;

		file.write(x_union.joyConfig_bytes, sizeof(x_union.joyConfig_bytes));
		file.write(y_union.joyConfig_bytes, sizeof(y_union.joyConfig_bytes));
		file.close();
	}
	else {
		//failed to create file
		Serial.println("Failed (hint: path must start with '/') ");
		Serial.print("errnum = ");
		Serial.println(file.errnum);
	}
			
	setVibe(3, 5, 5);
	Serial.println();
	Serial.println("JOYSTICK CALIBRATION COMPLETE");
	printJoystickCalibration();
	calibrate = false;	
}

void printJoystickCalibration() {
	Serial.println();
	Serial.println("Joystick Calibration Pulled from Memory:");
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
	Serial.println();
}

//if deadman is pressed read values, else reset all values
bool getInputs() {
	if (!digitalRead(deadmanPin)) {
		//read new values
		newValues.deadman = digitalRead(deadmanPin);
		newValues.button1 = digitalRead(button1Pin);
		newValues.button2 = digitalRead(button2Pin);
		newValues.xAxis = scaleAxis(xAxis, analogRead(xAxisPin));
		newValues.yAxis = scaleAxis(yAxis, analogRead(yAxisPin));
		//newValues.ocuBatt = batt;
#ifdef debug
		printValues();
#endif // DEBUG
		return true;
	} else {		
		initializeData(newValues);
		//newValues.ocuBatt = batt;
		return false;
	}
}

//measure battery voltage, convert to percent
void getBatt() {
	int raw;	
	analogReference(AR_INTERNAL_3_0);	// Set the analog reference to 3.0V (default = 3.6V)
	analogReadResolution(12); // Set the resolution to 12-bit (0..4095), Can be 8, 10, 12 or 14						  
	delay(1);	// Let the ADC settle	
	raw = analogRead(battVdcPin);	// Get the raw 12-bit, 0..3000mV ADC value	
	analogReference(AR_DEFAULT);	// Set the ADC back to the default settings
	analogReadResolution(10);
	raw = raw * VBAT_MV_PER_LSB;

	uint8_t battery_level;
	if (raw >= 3000) {
		battery_level = 100;
	} else if (raw > 2900) {
		battery_level = 100 - ((3000 - raw) * 58) / 100;
	} else if (raw > 2740) {
		battery_level = 42 - ((2900 - raw) * 24) / 160;
	} else if (raw > 2440) {
		battery_level = 18 - ((2740 - raw) * 12) / 300;
	} else if (raw > 2100) {
		battery_level = 6 - ((2440 - raw) * 6) / 340;
	} else {
		battery_level = 0;
	}

	batt = battery_level;
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
			return (int)map(val, joy.min, joy.center - joy.deadband, 0, 511);
		}
	} else {
		return 512;
	}
}

void encryptMsg(nRF24Msg_union *_msg) {
	randomSeed(millis());
	//Serial.print("Encrypt seed = ");
	for (uint8_t i = 0; i < 8; i++) {
		_msg->controlMsg_struct.seed[i] = (byte)random(0, 255);
		//Serial.print(_msg->controlMsg_struct.seed[i]);
		//Serial.print(", ");
	}

	/*Serial.print("Unencrypted msg = ");
	for (uint8_t i = 0; i < sizeof(_msg->sendMsg_bytes); i++) {
		Serial.print(_msg->sendMsg_bytes[i]);
		Serial.print(", ");
	}
	Serial.println();*/

	//Serial.println();
	byte msgCrypt[32];
	chacha.setIV(cypher.iv, chacha.ivSize());
	if (!chacha.setCounter(_msg->controlMsg_struct.seed, sizeof(_msg->controlMsg_struct.seed))) {
		Serial.println("Failed to set counter!");
	}
	chacha.encrypt(msgCrypt, _msg->msg_bytes, sizeof(_msg->msg_bytes));
	//Serial.print("Post Encrypt seed = ");
	for (uint8_t i = 0; i < 8; i++) {
		msgCrypt[i] = _msg->controlMsg_struct.seed[i];
		//Serial.print(msgCrypt[i]);
		//Serial.print(", ");
	}
	//Serial.println();
	//copy encrypted data back into union
	memcpy(_msg->msg_bytes, msgCrypt, sizeof(_msg->msg_bytes));	
}

bool sendData(nRF24Msg_union *_msg) {	

	//check encryption seed for debugging
	/*Serial.print("Pre send seed = ");
	for (uint8_t i = 0; i < 8; i++) {
		Serial.print(_msg->heartbeatMsg_struct.seed[i]);
		Serial.print(", ");
	}
	Serial.println();

	Serial.print("Encrypted msg = ");
	for (uint8_t i = 0; i < sizeof(_msg->sendMsg_bytes); i++) {
		Serial.print(_msg->sendMsg_bytes[i]);
		Serial.print(", ");
	}
	Serial.println();*/

	//Serial.print("Message type is = ");
	//Serial.println((char)_msg->heartbeatMsg_struct.msgType);

	//Serial.println();
	radio.stopListening();
	if (!radio.write(&_msg->msg_bytes, sizeof(_msg->msg_bytes))) {
		Serial.println("Failed to send message");
		if (radio.failureDetected) {
			Serial.print("radio hardware failure detected: ");
			//Serial.print(radio.get_status());
		}
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

//Checks for connection every 3sec
//Single short vibe while no connection
//Long vibe once connection is made
bool establishConnection() {
	Serial.println("Waiting for connection");
	int count = 0;
	nRF24Msg_union send = {};
	send.heartbeatMsg_struct = heartbeat;
	encryptMsg(&send);

	/*Serial.print("Pre send seed = ");
	for (uint8_t i = 0; i < 8; i++) {
		Serial.print(send.heartbeatMsg_struct.seed[i]);
		Serial.print(", ");
	}
	Serial.println();*/

	while (!sendData(&send)) {
		if (count < 20) {
			Serial.print(".");
			count++;
		} else {
			count = 0;
			Serial.println("!");
		}
		setVibe();
		encryptMsg(&send);

		/*Serial.print("Pre send seed = ");
		for (uint8_t i = 0; i < 8; i++) {
			Serial.print(send.heartbeatMsg_struct.seed[i]);
			Serial.print(", ");
		}
		Serial.println();*/

		delay(3000);
	}
	Serial.println(" ");
	Serial.println("Connection Established");
	setVibe(1, 5);

	return true;
}

bool ocuCalibrateRequest() {
	if (digitalRead(deadmanPin) && !digitalRead(button1Pin) && !digitalRead(button2Pin)) {
		if (!calibrate) {
			calibrate = true;
			calibrate_time = millis();
		}
	} else {
		calibrate = false;
	}
	if (calibrate && millis() - calibrate_time > calibrate_timeout) {
		return true;
	}
	return false;
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
		setVibe();
		return true;
	} else {
		//Stick here until reset		
		while (1) {
			Serial.println("Controller Failed Self-check");
			Serial.println("Restart or Try Calibrating Joystick");
			setVibe(3, 2, 2);			
			if (ocuCalibrateRequest()) {
				calibrateJoystick();
				if (selfCheck()) {
					return true;
				}
			}
			delay(3000);
		}
	}
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

