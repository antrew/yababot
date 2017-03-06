#include "Motor.h"

Motor::Motor(int enablePin, int backwardPin, int forwardPin) {
	this->enablePin = enablePin;
	this->backwardPin = backwardPin;
	this->forwardPin = forwardPin;
	pinMode(enablePin, OUTPUT);
	pinMode(backwardPin, OUTPUT);
	pinMode(forwardPin, OUTPUT);
}

void Motor::setDirection(float direction) {
	//limit the target value to the range -1 .. 1
	if (direction > 1)
		direction = 1;
	if (direction < -1)
		direction = -1;

	uint8_t power = abs(direction * 255);

	if (direction < 0) {
		digitalWrite(backwardPin, HIGH);
		digitalWrite(forwardPin, LOW);
	} else {
		digitalWrite(backwardPin, LOW);
		digitalWrite(forwardPin, HIGH);
	}
	analogWrite(enablePin, power);
}

Motor::~Motor() {
}
