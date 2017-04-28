#ifndef OCU_h
#define OCU_h

#include <bluefruit.h>
#include <Nffs.h>
#include "WOLF_COMS.h"

class OCU {
public:
	OCU();
	void begin();
	bool calibrateRequested();
	void calibrateJoysticks();
	void printJoystickCalibration();
	bool readInputs(WOLF_COMS::controlMsg*);
	byte readBatt();
	void setVibe(int numPulses = 1, int pulseDuration = 1, int pulseDelay = 1);
	void vibe();
	void printInputValues(WOLF_COMS::controlMsg *values);
	~OCU();

private:
	//#define FILENAME "/config.txt"
	const char FILENAME[12] = "/config.txt";
	const float VBAT_MV_PER_LSB = 0.73242188F;  // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
	
	//pin declarations
	const uint8_t xAxisPin = A0;
	const uint8_t yAxisPin = A1;
	const uint8_t battVdcPin = A7;
	const uint8_t button1Pin = PIN_A2;
	const uint8_t button2Pin = PIN_A3;
	const uint8_t deadmanPin = 15;
	const uint8_t vibePin = 16;

	typedef struct _joystick {
		int center; // = 450;
		int min;	// = 0;
		int max;	// = 920;
		int deadband; // = 40;
	}joystick;

	typedef union _joyConfig_union {
		joystick joyConfig_struct;
		uint8_t joyConfig_bytes[sizeof(joyConfig_struct)];
	}joyConfig_union;

	const int joyCenterDefault = 450;
	const int joyMinDefault = 0;
	const int joyMaxDefault = 920;
	const int joyDeadbandDefault = 40;

	//Vibe interrupt variables
	int vibePulses = 0;
	int vibeDuration = 100;
	int vibeDurationCounter = 100;
	int vibeDelay = 100;
	int vibeDelayCounter = 100;

	bool calibrate = false;
	unsigned long calibrate_time = 0;
	const int calibrate_timeout = 3000;

	joystick xAxis, yAxis = {};
	NffsFile file;

	void initializeJoysticks();
	void validityCheckJoystickConfig(joyConfig_union*, joystick*);
	int scaleAxis(joystick, int);
	bool selfCheck();
};

#endif //OCU.h

