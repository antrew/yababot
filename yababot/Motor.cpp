#include "Motor.h"

Motor::Motor(int backwardPin, int forwardPin) {
	this->backwardPwmPin = backwardPin;
	this->forwardPwmPin = forwardPin;
}

void Motor::setDirection(float direction) {
	//limit the target value to the range -1 .. 1
	if (direction > 1)
		direction = 1;
	if (direction < -1)
		direction = -1;

	uint8_t power = abs(direction * 255);

	if (direction < 0) {
		analogWrite(backwardPwmPin, power);
		analogWrite(forwardPwmPin, LOW);
	} else {
		analogWrite(backwardPwmPin, LOW);
		analogWrite(forwardPwmPin, power);
	}
}

Motor::~Motor() {
}
