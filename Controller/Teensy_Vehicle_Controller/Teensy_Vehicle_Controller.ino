/*
 Name:		Teensy_Vehicle_Controller.ino
 Created:	4/10/2017 11:04:10
 Author:	Travis.rideout
*/

//TODO: Scan for best channel on startup and tell controller radio to switch to that channel
//TODO: implement ID pairing check
//TODO: CAN Bus

#include "Teensy_Vehicle_Controller.h"

void setup() {
	Serial.begin(115200);
	printf_begin();

	//Print header
	Serial.println("HDT ROBOTICS");
	Serial.println("SMET RADIO CAN TRANSLATOR");
	Serial.println("VERSION: 0.1");
	Serial.println();
	
	//Set pin modes	
	pinMode(ledRedPin, OUTPUT);
	pinMode(ledGreenPin, OUTPUT);
	pinMode(ledBluePin, OUTPUT);
	//pinMode(clearFaultPin, INPUT_PULLUP);

	// Setup and configure rf radio
	radio.begin();
	radio.setPALevel(RF24_PA_MIN);
	radio.setAutoAck(1);                    // Ensure autoACK is enabled
	radio.enableAckPayload();               // Allow optional ack payloads
	radio.setRetries(8, 10);                 // Smallest time between retries, max no. of retries
	radio.openWritingPipe(pipes[1]);        // Both radios listen on the same pipes by default, and switch when writing
	radio.openReadingPipe(1, pipes[0]);
	radio.startListening();                 // Start listening
	radio.printDetails();                   // Dump the configuration of the rf unit for debugging

	// Initialise AHRS sensor
	if (!bno.begin()) {
		// There was a problem detecting the BNO055 ... check connections
		Serial.print("AHRS not detected ... Check wiring!");		
	} else {
		displayAHRSDetails();
		AHRS_active = true;
		bno.setExtCrystalUse(true);
	}

	//initialize data
	initializenRF24Messages();
	initializeOutputs();

	//initialize encryption
	chacha.setNumRounds(cypher.rounds);
	chacha.setKey(cypher.key, cypher.keySize);
	chacha.setIV(cypher.iv, chacha.ivSize());

	prev_time = millis();
	establishConnection();
}

void loop() {
	if (radio.available()) {
		byte pipeNo;
		while (radio.available(&pipeNo)) {
			//decide what payload to put in ack 
			if (1) {
				nRF24Msg_union ackMsg = {};
				ackMsg.guiMsg_struct = gui;
				readAHRS();
				Serial.println("sending Ack Packet");
				//store next ack payload to go out when a msg read occurs
				radio.writeAckPayload(pipeNo, &ackMsg.msg_bytes, sizeof(ackMsg.msg_bytes));
			}
			
			nRF24Msg_union msg = {};
						
			byte msgCrypt[sizeof(msg)];	
			radio.read(&msgCrypt, sizeof(msg));      // Get the payload	
			
			/*Serial.print("encrypted msg = ");
			for (uint8_t i = 0; i < sizeof(msg); i++) {
				Serial.print(msgCrypt[i]);
				Serial.print(", ");
			}
			Serial.println();*/

			byte seed[8];

			//Serial.print("Decrypt seed = ");
			for (uint8_t i = 0; i < 8; i++) {
				seed[i] = msgCrypt[i];
				//Serial.print(seed[i]);
				//Serial.print(", ");
			}
			//Serial.println();

			chacha.setIV(cypher.iv, chacha.ivSize());
			if (!chacha.setCounter(seed, sizeof(seed))) {
				Serial.println("Failed to set decrypt counter!");
			}
			chacha.decrypt(msg.msg_bytes, msgCrypt, sizeof(msgCrypt));

			/*Serial.print("msg = ");
			for (uint8_t i = 0; i < sizeof(msg); i++) {
				Serial.print(msg.msg_bytes[i]);
				Serial.print(", ");
			}
			Serial.println();*/

			parseMessage(&msg);
		}
		if (fault_state && faultCode == Loss_Of_Signal) {
			if (establishConnection()) {
				Serial.println("Connection Re-Established - Fault Self Cleared");
				clearFault();
			}
		}
		//parseMessage(&msg);
		if (!fault_state) {
			setOutputs();
		}
		prev_time = millis();
	}
	else if (millis() - prev_time > heartbeat_timeout && !fault_state) {
		prev_time = millis();
		setFault(Loss_Of_Signal);
	}
	//if (fault_state) {
	//	if (digitalRead(clearFaultPin) == 0) {
	//		unsigned long pressedTime = millis();
	//		bool clearButtonDown = true;
	//		while (clearButtonDown) {
	//			if (millis() - pressedTime > clearFaultTimer) {
	//				Serial.println("Fault Cleared");
	//				clearFault();
	//				clearButtonDown = false;
	//			}
	//			else {
	//				clearButtonDown = !digitalRead(clearFaultPin);
	//			}
	//			delay(50);	//debounce
	//		}
	//	}
	//	if (millis() - prev_time > faultPrintTimer) {
	//		printFault();
	//		prev_time = millis();
	//	}
	//}
}

void displayAHRSDetails(void) {
	sensor_t sensor;
	bno.getSensor(&sensor);
	Serial.println("------------------------------------");
	Serial.print("Sensor:       "); Serial.println(sensor.name);
	Serial.print("Driver Ver:   "); Serial.println(sensor.version);
	Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
	Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
	Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
	Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
	Serial.println("------------------------------------");
	Serial.println("");
	delay(500);
}

//initialize data struct to zero values
void initializenRF24Messages() {
	control.pairedID = 201;
	control.msgType = 'C';
	control.xAxis = 512;
	control.yAxis = 512;
	control.button1 = false;
	control.button2 = false;
	control.deadman = false;
	control.ocuBatt = 100;

	heartbeat.msgType = 'H';
	gui.msgType = 'G';
	fault.msgType = 'F';
}

//initialize output object values
void initializeOutputs() {
	led.red = 0;
	led.green = 0;
	led.blue = 0;

	setOutputs();
}

//checks for connection every 3sec
bool establishConnection() {
	Serial.println("Waiting for connection");
	long startTime = millis();
	int count = 0;
	bool ledBlinkState = false;
	int blinkrate = 1000;
	long localtime = millis();
	//flush rx buffer first
	if (radio.available()) {
		controlMsg buf;
		radio.read(&buf, sizeof(buf));
		Serial.println("Flushing buffer");
	}
	while (!radio.available()) {
		if (millis() - localtime > blinkrate) {
			localtime = millis();
			if (count < 20) {
				Serial.print(".");
				count++;
			}
			else {
				count = 0;
				Serial.println("!");
			}
			ledBlinkState = !ledBlinkState;
			if (ledBlinkState) {
				led.blue = 125;
			}
			else {
				led.blue = 0;
			}
		}
		/*if (millis() - startTime > 60000) {
			Serial.println(" ");
			Serial.println("Unable to Establish a Connection");
			setFault(Unable_To_Establish_Connection);
			return false;
		}*/
		analogWrite(ledBluePin, led.blue);
		delay(50);
	}
	Serial.println(" ");
	Serial.println("Connection Established");
	led.blue = 0;
	led.green = 125;
	analogWrite(ledBluePin, led.blue);
	analogWrite(ledGreenPin, led.green);
	return true;
}

void parseMessage(nRF24Msg_union *msg) {
	Serial.print("Message Type is: ");
	Serial.println((char)msg->heartbeatMsg_struct.msgType);

	switch (msg->heartbeatMsg_struct.msgType){
	case 'H':
		Serial.print("Heartbeat message recieved: ");
		Serial.println(msg->heartbeatMsg_struct.count);
		break;
	case 'C':
		//data validity check?
		//pass to CAN bus
		Serial.println("Control message recieved: ");
		//print controls message data for debug
		break;
	case 'U':
		//data validity check?
		//pass to CAN bus
		Serial.println("User request message recieved: ");
		//print controls message data for debug
		break;
	default:
		//fault or ignore?
		Serial.println("Invalid message data type");
		break;
	}
	

	//data validity check?
	//if (newValues.xAxis > 1023 || newValues.xAxis < 0 || newValues.yAxis > 1023 || newValues.yAxis < 0) {
	//	//setFault(Input_Out_Of_Range);
	//	Serial.print("Input_Out_Of_Range Fault Detected: ");
	//	Serial.print("xAxis = ");
	//	Serial.print(newValues.xAxis);
	//	Serial.print(", yAxis = ");
	//	Serial.println(newValues.yAxis);

	//	//trying this instead of set fault
	//	initializeData(newValues);
	//	initializeOutputs();
	//	return;
	//}		
}

void setOutputs() {
	analogWrite(ledRedPin, led.red);
	analogWrite(ledGreenPin, led.green);
	analogWrite(ledBluePin, led.blue);
}

void printOutputs() {	
	Serial.print(led.red);
	Serial.print(", ");
	Serial.print(led.green);
	Serial.print(", ");
	Serial.print(led.blue);
	Serial.print(", ");
	Serial.print(control.pairedID);
	Serial.print(", ");
	Serial.print((char)control.msgType);
	Serial.print(", ");
	Serial.println(control.ocuBatt);
}

void setFault(faultCodes code) {
	fault_state = true;
	faultCode = code;
	initializenRF24Messages();
	initializeOutputs();
	led.red = 125;
	led.green = 0;
	led.blue = 0;
	setOutputs();
	printFault();
}

void printFault() {
	Serial.print("Fault: ");
	Serial.println(faultCode);
}

void clearFault() {
	fault_state = false;
	faultCode = No_Fault;
	initializenRF24Messages();
	led.red = 0;
	led.green = 0;
	led.blue = 0;
	setOutputs();
	establishConnection();
}

void readAHRS() {
	/* Get a new sensor event */
	sensors_event_t event;
	bno.getEvent(&event);

	gui.orient[0] = (int)(event.orientation.y * 100);
	gui.orient[1] = (int)(event.orientation.z * 100);
	gui.orient[2] = (int)(event.orientation.x * 100);
}

