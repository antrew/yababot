#include <SPI.h>
#include <printf.h>
//#include <string.h>
#include "RF24.h"
#include "LiquidCrystal_I2C.h"
#include "JoystickShield.h"
#include "../common/common.h"

#define RADIO_PIN_CE 9
#define RADIO_PIN_CS 10

const double PID_P = 0.15;
const double PID_I = 0.000001;
const double PID_D = 0.002;

RF24 radio(RADIO_PIN_CE, RADIO_PIN_CS);
LiquidCrystal_I2C lcd(0x27, 16, 2);
JoystickShield joystickShield;

boolean isInSettingsMode = false;

const char * FW_ID = "joystick";

char line[30];
unsigned long counter = 0;

void onSettingsButton() {
	lcd.setCursor(0, 0);
	if (isInSettingsMode) {
		Serial.println("exiting settings mode");
		lcd.print("settings");
	} else {
		Serial.println("entering settings mode");
		lcd.print("operation");
	}
	isInSettingsMode = !isInSettingsMode;
}

void sendInitialMessage() {
	struct radioMessage message;
	message.timestamp = micros();
	message.counter = counter++;
	message.forward = 0;
	message.command = SET_PID_COEFFICIENTS;
	message.pidP = PID_P;
	message.pidI = PID_I;
	message.pidD = PID_D;

	Serial.print("Sending initial message... ");
	if (!radio.write(&message, sizeof(message))) {
		Serial.println(F("failed"));
	}
	Serial.println(F("done"));
}

void setup() {
	Serial.begin(115200);
	printf_begin();
	Serial.println(FW_ID);

	radio.begin();

	// Set the PA Level low to prevent power supply related issues since this is a
	// getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
	radio.setPALevel(RADIO_POWER_LEVEL);

	// increase range by reducing the speed
	radio.setDataRate(RADIO_DATA_RATE);
	// 108 - 2.508 Ghz - Above most Wifi Channels
	radio.setChannel(108);

	radio.openWritingPipe(RADIO_ADDRESS);

	radio.printDetails();

	lcd.begin();
	lcd.backlight();
	lcd.clear();
	lcd.print(FW_ID);

	joystickShield.setButtonPins(2, 3, 4, 5, 6, 8, 7);
	joystickShield.setButtonPinsUnpressedState(HIGH, LOW, LOW, LOW, LOW, LOW,
	LOW);

	joystickShield.calibrateJoystick();

	joystickShield.onEButton(&onSettingsButton);
	joystickShield.onFButton(&onSettingsButton);
	joystickShield.onDownButton(&onSettingsButton);
	joystickShield.onUpButton(&onSettingsButton);
	joystickShield.onLeftButton(&onSettingsButton);
	joystickShield.onRightButton(&onSettingsButton);

	sendInitialMessage();

}

void loop() {

	//joystickShield.processEvents();
	joystickShield.processCallbacks();
	int joystickX = joystickShield.xAmplitude();
	int joystickY = joystickShield.yAmplitude();

	struct radioMessage message;
	message.timestamp = micros();
	message.counter = counter++;
	message.forward = joystickY;
	message.rotate = joystickX;
	message.command = NONE;
	if (joystickShield.isEButton()) {
		Serial.println("E button pressed. Toggling motors. Waiting for the button to be released...");
		message.command = TOGGLE_MOTORS;
		while (joystickShield.isEButton()) {
			// wait until the user releases the button
			joystickShield.processEvents();
		}
	} else if (joystickShield.isFButton()) {
		Serial.println("F button pressed. Calibrating. Waiting for the button to be released...");
		message.command = CALIBRATE;
		while (joystickShield.isFButton()) {
			// wait until the user releases the button
			joystickShield.processEvents();
		}
	}

	Serial.print("Sending ");
	Serial.print(message.timestamp);
	Serial.print("... ");
	if (!radio.write(&message, sizeof(message))) {
		Serial.println(F("failed"));
	}
	Serial.println(F("done"));

	sprintf(line, "x=%4d ; y=%4d", joystickX, joystickY);
//	lcd.setCursor(0, 0);
//	lcd.print(line);
//
	Serial.println(line);

//	delay(1000);
} // Loop
