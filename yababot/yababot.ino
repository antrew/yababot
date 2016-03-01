/*
 * Getting Started example sketch for nRF24L01+ radios
 * This is a very basic example of how to send data from one node to another
 * Updated: Dec 2014 by TMRh20
 */

#include <SPI.h>
#include <printf.h>
#include "RF24.h"
#include "common.h"

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(9, 10);
/**********************************************************/

void setup() {
	Serial.begin(115200);
	printf_begin();
	Serial.println("yababot.ino");

	radio.begin();

	// Set the PA Level low to prevent power supply related issues since this is a
	// getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
	radio.setPALevel(RF24_PA_MIN);

	// increase range by reducing the speed
	radio.setDataRate(RF24_250KBPS);
	// 108 - 2.508 Ghz - Above most Wifi Channels
	radio.setChannel(108);

	radio.openReadingPipe(1, RADIO_ADDRESS);

	// Start the radio listening for data
	radio.startListening();

	radio.printDetails();
}

void loop() {

	unsigned long got_time;

	if (radio.available()) {
		// Variable for the received timestamp
		while (radio.available()) {             // While there is data ready
			radio.read(&got_time, sizeof(unsigned long)); // Get the payload
		}

		Serial.print("received: ");
		Serial.println(got_time);
	}

} // Loop

