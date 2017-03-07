#ifndef MOTOR_H_
#define MOTOR_H_

#include <Arduino.h>

class Motor {
public:
	Motor(int enablePin, int backwardPin, int forwardPin);
	void on();
	void off();
	void setDirection(float target);
	virtual ~Motor();
private:
	int enablePin;
	int backwardPin;
	int forwardPin;
	bool motorEnabled;
};

#endif /* MOTOR_H_ */
