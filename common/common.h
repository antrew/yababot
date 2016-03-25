// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _common_H_
#define _common_H_
#include "Arduino.h"
//add your includes for the project common here

//end of add your includes here

//add your function definitions for the project common here

uint8_t RADIO_ADDRESS[6] = "YABAB";

// Set the PA Level low to prevent power supply related issues since this is a
// getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
#define RADIO_POWER_LEVEL RF24_PA_MIN

// increase range by reducing the speed
#define RADIO_DATA_RATE RF24_250KBPS

// 108 - 2.508 Ghz - Above most Wifi Channels
#define RADIO_CHANNEL 108

struct radioMessage {
	unsigned long timestamp;
	unsigned long counter;
	int8_t forward;
};

//Do not add code below this line
#endif /* _common_H_ */
