/*
 * Getting Started example sketch for nRF24L01+ radios
 * This is a very basic example of how to send data from one node to another
 * Updated: Dec 2014 by TMRh20
 */

#include <SPI.h>
#include <printf.h>
#include "RF24.h"
#include "../common/common.h"
#include "Motor.h"

// 2 and 3 are the only two pins supporting hardware interrupts
const uint8_t LEFT_ENCODER_INTERRUPT_PIN = 2;
const uint8_t RIGHT_ENCODER_INTERRUPT_PIN = 3;

const uint8_t LEFT_ENCODER_SECOND_PIN = 4;
const uint8_t RIGHT_ENCODER_SECOND_PIN = A0;

const uint8_t MOTOR_LEFT_FORWARD = 5;
const uint8_t MOTOR_LEFT_BACKWARD = 6;

const uint8_t MOTOR_RIGHT_FORWARD = 9;
const uint8_t MOTOR_RIGHT_BACKWARD = 10;

const uint8_t RADIO_CE_PIN = 7;
const uint8_t RADIO_CS_PIN = 8;
// RADIO MOSI = 11
// RADIO MISO = 12
// RADIO SCK  = 13

const char * FW_ID = "yababot";

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);
/**********************************************************/

Motor leftMotor = Motor(MOTOR_LEFT_BACKWARD, MOTOR_LEFT_FORWARD);
Motor rightMotor = Motor(MOTOR_RIGHT_BACKWARD, MOTOR_RIGHT_FORWARD);

void setupMotors() {
//	pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
//	pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
//	pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
//	pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);

}

void setup() {
	Serial.begin(115200);
	printf_begin();
	Serial.println(FW_ID);

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

	setupMotors();
}

void loop() {

	struct radioMessage message;

	if (radio.available()) {
		// Variable for the received timestamp
		while (radio.available()) {             // While there is data ready
			radio.read(&message, sizeof(message)); // Get the payload
		}

		Serial.print("received: {");
		Serial.print(message.timestamp);
		Serial.print(", ");
		Serial.print(message.counter);
		Serial.print(", ");
		Serial.print(message.forward);
		Serial.print("}");
		Serial.println();

		float direction = message.forward / 100.0;
		leftMotor.setDirection(direction);
		rightMotor.setDirection(direction);
	}

} // Loop
