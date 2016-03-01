/*
 * Getting Started example sketch for nRF24L01+ radios
 * This is a very basic example of how to send data from one node to another
 * Updated: Dec 2014 by TMRh20
 */

#include <SPI.h>
#include <printf.h>
#include "RF24.h"
#include "common.h"
#include <LiquidCrystal_I2C.h>

#define RADIO_PIN_CE 9
#define RADIO_PIN_CS 10

RF24 radio(RADIO_PIN_CE, RADIO_PIN_CS);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
	Serial.begin(115200);
	printf_begin();
	Serial.println("joystick.ino");

	radio.begin();

	// Set the PA Level low to prevent power supply related issues since this is a
	// getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
	radio.setPALevel(RADIO_POWER_LEVEL);

	// increase range by reducing the speed
	radio.setDataRate(RF24_250KBPS);
	// 108 - 2.508 Ghz - Above most Wifi Channels
	radio.setChannel(108);

	radio.openWritingPipe(RADIO_ADDRESS);

	radio.printDetails();


    lcd.begin();
    lcd.backlight();
    lcd.clear();
	lcd.print("Hello world!");
}

void loop() {

	unsigned long start_time = micros();

	Serial.print("Sending ");
	Serial.print(start_time);
	Serial.print("... ");
	if (!radio.write(&start_time, sizeof(unsigned long))) {
		Serial.println(F("failed"));
	}
	Serial.println(F("done"));

	delay(1000);
} // Loop
