/*
 * ComplementaryFilter.cpp
 *
 *  Created on: 23.08.2015
 *      Author: andy
 */

#include "ComplementaryFilter.h"
#include <Arduino.h>
#include <math.h>

ComplementaryFilter::ComplementaryFilter() {
}

ComplementaryFilter::~ComplementaryFilter() {
}

void ComplementaryFilter::calculateDt() {
	// calculate dt based on the current time and the previous measurement time
	unsigned long currentTime = micros();
	dt = (currentTime - lastTime) / 1000000.0;
	lastTime = currentTime;
}

void ComplementaryFilter::updateValue(
		double accelerometerAngle,
		double gyroRate) {

	calculateDt();

	//complementary filter

	// T defines the averaging period for accelerometerAngle
	// bigger T increases the use of the gyroscope over the accelerometer
	// smaller T increases the use of the accelerometer over the gyroscope
	// use Arduino Serial Plotter over accelerometerAngle and angle to tune T
	double T = 1; // in seconds
	double K = T / (T + dt);
	double gyroscopeAngle = gyroRate * dt;
	angle = K * (angle + gyroscopeAngle) + (1 - K) * accelerometerAngle;

	Serial.print(" accelerometerAngle ");
	Serial.print(accelerometerAngle);

//	Serial.print(" dt = ");
//	Serial.print(dt);
//	Serial.print(" K = ");
//	Serial.print(K);

	Serial.print(" filtered angle ");
	Serial.print(angle);

	Serial.println();
}

double ComplementaryFilter::getAngle() {
	return this->angle;
}

double ComplementaryFilter::getDt() {
	return this->dt;
}
