#ifndef MOTOR_H_
#define MOTOR_H_

#include <Arduino.h>

class Motor {
public:
	Motor(int backwardPin, int forwardPin);
	void setDirection(float target);
	virtual ~Motor();
private:
	int backwardPwmPin;
	int forwardPwmPin;
};

#endif /* MOTOR_H_ */
