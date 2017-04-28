#include "OCU.h"

//TODO: Disallow deadman initialization with joystick not centered or buttons pressed

OCU::OCU() {
}

void OCU::begin() {
	//Set pin modes
	pinMode(xAxisPin, INPUT);
	pinMode(yAxisPin, INPUT);
	pinMode(button1Pin, INPUT_PULLUP);	//low, 0, is pressed. High, 1, is not pressed
	pinMode(button2Pin, INPUT_PULLUP);	//low, 0, is pressed. High, 1, is not pressed
	pinMode(deadmanPin, INPUT_PULLUP);	//low, 0, is pressed. High, 1, is not pressed
	pinMode(vibePin, OUTPUT);

	// Bluefruit module must be initialized for Nffs to work
	Bluefruit.begin();	
	Nffs.begin();
	initializeJoysticks();
	selfCheck();
	readBatt();
}

bool OCU::calibrateRequested() {
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

//Two step process to get center/min/max values of joystick range
void OCU::calibrateJoysticks() {
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

	Serial.print("Open ");
	Serial.print(FILENAME);
	Serial.print(" file to write ... ");

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
	} else {
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

void OCU::printJoystickCalibration() {
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

//if deadman is pressed read values
bool OCU::readInputs(WOLF_COMS::controlMsg *values) {
	if (!digitalRead(deadmanPin)) {
		//read new values
		values->deadman = digitalRead(deadmanPin);
		values->button1 = digitalRead(button1Pin);
		values->button2 = digitalRead(button2Pin);
		values->xAxis = scaleAxis(xAxis, analogRead(xAxisPin));
		values->yAxis = scaleAxis(yAxis, analogRead(yAxisPin));
#ifdef debug
		printInputValues(values);
#endif // DEBUG
		return true;
	} 
	return false;
}

//measure battery voltage, convert to percent
byte OCU::readBatt() {
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

	return battery_level;
}

//Set a vibe command.  Call this on a delay to size durations 
void OCU::setVibe(int numPulses, int pulseDuration, int pulseDelay) {
	vibePulses = numPulses;
	vibeDuration = pulseDuration;
	vibeDurationCounter = pulseDuration;
	vibeDelay = pulseDelay;
	vibeDelayCounter = pulseDelay;
}

void OCU::vibe() {
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
}

void OCU::printInputValues(WOLF_COMS::controlMsg *values) {
	Serial.print(values->xAxis);
	Serial.print(", ");
	Serial.print(values->yAxis);
	Serial.print(", ");
	Serial.print(values->button1);
	Serial.print(", ");
	Serial.print(values->button2);
	Serial.print(", ");
	Serial.println(values->deadman);
}

//open config file, error if not found -> force calibrate, create config file there
//Pull values from config file, validity check, initialize working variables
void OCU::initializeJoysticks() {
	file.open(FILENAME, FS_ACCESS_READ);
	if (file.exists()) {
		Serial.print(FILENAME);
		Serial.println(" file opened");

		//create byte unions of joystick structs to write to
		joyConfig_union x_union = {};
		joyConfig_union y_union = {};

		//read from file and store in unions
		file.read(x_union.joyConfig_bytes, sizeof(x_union.joyConfig_bytes));
		file.read(y_union.joyConfig_bytes, sizeof(y_union.joyConfig_bytes));

		validityCheckJoystickConfig(&x_union, &xAxis);
		validityCheckJoystickConfig(&y_union, &yAxis);
		
	} else {
		//no config file was found so creating a new one with default values
		Serial.print("Open ");
		Serial.print(FILENAME);
		Serial.print(" file to write ... ");

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

void OCU::validityCheckJoystickConfig(joyConfig_union *src, joystick *dst) {
	//validity check values and set 
	//TODO: pull max/min from header set values not hard coded
	if (src->joyConfig_struct.center > 0 && src->joyConfig_struct.center < 1023) {
		dst->center = src->joyConfig_struct.center;
	} else {
		//error
	}
	if (src->joyConfig_struct.min > 0 && src->joyConfig_struct.min < 1023) {
		dst->min = src->joyConfig_struct.min;
	} else {
		//error
	}
	if (src->joyConfig_struct.max > 0 && src->joyConfig_struct.max < 1023) {
		dst->max = src->joyConfig_struct.max;
	} else {
		//error
	}
	if (src->joyConfig_struct.deadband > 0 && src->joyConfig_struct.deadband < 1023) {
		dst->deadband = src->joyConfig_struct.deadband;
	} else {
		//error
	}	
}

//Scale a joystick axis reading between its min/max around its center +/- deadband
int OCU::scaleAxis(joystick joy, int val) {
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

//Check that the joystick is within centered range and no buttons are shorted
//Allow joystick calibration
bool OCU::selfCheck() {
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
		//TODO: make this non-blocking to allow BT traffic
		while (1) {
			Serial.println("Controller Failed Self-check");
			Serial.println("Restart or Try Calibrating Joystick");
			setVibe(3, 2, 2);
			if (calibrateRequested()) {
				calibrateJoysticks();
				if (selfCheck()) {
					return true;
				}
			}
			delay(3000);
		}
	}
}

OCU::~OCU() {
}
