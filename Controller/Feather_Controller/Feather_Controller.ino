/*
 Name:		Feather_Controller.ino
 Created:	4/5/2017 09:57:14
 Author:	Travis.rideout
*/

//TODO: No EEPROM so need to initialize values somehow else. nffs?
//TODO: add BLE for operator GUI
//TODO: loop timing needs to be re thought for high CPU frequency
//TODO: Implement rx from vehicle
//TODO: Create heartbeat specific message
//TODO: printf not working
//TODO: create error handler

//TODO: utilize sleep modes on no user interaction timeouts to save battery
//TODO: Programatically set radio PAlevel based on signal strength/lossed packets
//TODO: measure battery voltage
//TODO: Handshake with vehicle using ack and ID
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
	Serial.println("VERSION: 1.1");
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
	Serial.println("Init Radio");

	//initialize data
	initializeData(oldValues);
	initializeData(newValues);
	initializeData(heartbeat);
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
	if (digitalRead(deadmanPin) && !digitalRead(button1Pin) && !digitalRead(button2Pin)) {
		if (!calibrate) {
			calibrate = true;
			calibrate_time = millis();
		}
	} else {
		calibrate = false;
	}
	if (calibrate && millis() - calibrate_time > calibrate_timeout) {
		calibrateJoystick();
	}	

	//All nRF24 calls should be from here to avoid overlap
	if (millis() - prev_time > message_frequency && getInputs()) {
		prev_time = millis();
		generateSeed(newValues);
		if (!sendData(newValues)) {
			establishConnection();
		}
	} else if (millis() - prev_time > heartbeat_frequency) {
		prev_time = millis();
		Serial.print("sending heartbeat message: ");
		Serial.println(count);
		count++;
		generateSeed(heartbeat);
		if (!sendData(heartbeat)) {
			establishConnection();
		}
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

//initialize data struct to zero values
void initializeData(dataStruct &data) {
	//data.pairedID = 201
	data.xAxis = 512;
	data.yAxis = 512;
	data.button1 = true;
	data.button2 = true;
	data.deadman = true;
	//data.ocuBatt = 100;
}


void initializeJoysticks() {
	//open config file, error if not found -> force calibrate, create config file there
	
	file.open(FILENAME, FS_ACCESS_READ);
	if (file.exists()) {
		Serial.println(FILENAME " file opened");

		uint32_t readlen;
		char buffer[42] = { 0 };
		readlen = file.read(buffer, sizeof(buffer));

		buffer[readlen] = 0;
		Serial.println(buffer);
		int xc, xmi, xma, xd, yc, ymi, yma, yd;

		sscanf(buffer, "%d,%d,%d,%d,%d,%d,%d,%d", 
			&xAxis.center, &xAxis.min, &xAxis.max, &xAxis.deadband, 
			&yAxis.center, &yAxis.min, &yAxis.max, &yAxis.deadband);
		
		if (xc > 0 && xc < 1023) {
			xAxis.center = xc;
		} else {
			//error
		}
		if (xmi > 0 && xmi < 1023) {
			xAxis.min = xmi;
		} else {
			//error
		}
		if (xma > 0 && xma < 1023) {
			xAxis.max = xma;
		} else {
			//error
		}
		if (xcd > 0 && xd < 1023) {
			xAxis.deadband = xd;
		} else {
			//error
		}
		if (yc > 0 && yc < 1023) {
			yAxis.center = yc;
		} else {
			//error
		}
		if (ymi > 0 && ymi < 1023) {
			yAxis.min = ymi;
		} else {
			//error
		}
		if (yma > 0 && yma < 1023) {
			yAxis.max = yma;
		} else {
			//error
		}
		if (ycd > 0 && yd < 1023) {
			yAxis.deadband = yd;
		} else {
			//error
		}

	} else {
		Serial.print("Open " FILENAME " file to write ... ");

		if (file.open(FILENAME, FS_ACCESS_WRITE)) {
			Serial.println("OK");
			char initialValues[42] = "0450,0000,0920,0040,0450,0000,0920,0040,";
			file.write(initialValues, strlen(initialValues));
			file.close();
		} else {
			Serial.println("Failed (hint: path must start with '/') ");
			Serial.print("errnum = ");
			Serial.println(file.errnum);
		}
	}	
	printJoystickCalibration();
}

//Two step process to get center/min/max values of joystick range
calibrateJoystick() {
	Serial.println("JOYSTICK CALIBRATION STARTED");
	Serial.println("Leave Joystick Centered");
	setVibe(1, 5);
	xAxis.center = analogRead(xAxisPin);
	yAxis.center = analogRead(yAxisPin);
	xAxis.deadband = 40;
	yAxis.deadband = 40;
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

	//TODO: need leading 0's to get exactly 4bytes per number
	//Store values to config file
	String string1 = String(xAxis.center) + ",";
	String string2 = String(xAxis.min) + ",";
	String string3 = String(xAxis.max) + ",";
	String string4 = String(xAxis.deadband) + ",";
	String string5 = String(yAxis.center) + ",";
	String string6 = String(yAxis.min) + ",";
	String string7 = String(yAxis.max) + ",";
	String string8 = String(yAxis.deadband) + ",";

	String buf = string1 + string2 + string3 + string4 + string5 + string6
		+ string7 + string8;
	char config[42];	//adjust pubsub to accept messages of this length
	buf.toCharArray(config, sizeof(config));
	Serial.println(config);
		
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
		//shuffle values back into history
		shuffleValues(oldValues, newValues);
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
		shuffleValues(oldValues, newValues);
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

void generateSeed(dataStruct &data) {
	randomSeed(millis());
	for (uint8_t i = 0; i < 8; i++) {
		data.seed[i] = (byte)random(0, 255);
	}
}

bool sendData(dataStruct &data) {
	byte msg[sizeof(data)];
	byte msgCrypt[sizeof(data)];
	memcpy(msg, &data, sizeof(data));
	chacha.setIV(cypher.iv, chacha.ivSize());
	if (!chacha.setCounter(data.seed, sizeof(data.seed))) {
		Serial.println("Failed to set counter!");
	}
	chacha.encrypt(msgCrypt, msg, sizeof(msg));
	//Serial.print("Encrypt seed = ");
	for (uint8_t i = 0; i < 8; i++) {
		msgCrypt[i] = data.seed[i];
		//Serial.print(msgCrypt[i]);
		//Serial.print(", ");
	}
	//Serial.println();
	radio.stopListening();
	if (!radio.write(&msgCrypt, sizeof(msgCrypt))) {
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

//Set a vibe command. Runs on interrupt timer1. All times in multiples of 100 ms
void setVibe(int numPulses = 1, int pulseDuration = 1, int pulseDelay = 1) {
	vibePulses = numPulses;
	vibeDuration = pulseDuration;
	vibeDurationCounter = pulseDuration;
	vibeDelay = pulseDelay;
	vibeDelayCounter = pulseDelay;
}

//Checks for connection every 3sec
//Single short vibe while no connection
//Long vibe once connection is made
bool establishConnection() {
	Serial.println("Waiting for connection");
	int count = 0;
	generateSeed(heartbeat);
	while (!sendData(heartbeat)) {
		if (count < 20) {
			Serial.print(".");
			count++;
		} else {
			count = 0;
			Serial.println("!");
		}
		setVibe();
		generateSeed(heartbeat);
		delay(3000);
	}
	Serial.println(" ");
	Serial.println("Connection Established");
	setVibe(1, 5);

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
		setVibe();
		return true;
	} else {
		//Stick here until reset		
		while (1) {
			Serial.println("Controller Failed Self-check");
			Serial.println("Restart or Try Calibrating Joystick");
			setVibe(3, 2, 2);
			if (calibrateJoystick()) {
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

