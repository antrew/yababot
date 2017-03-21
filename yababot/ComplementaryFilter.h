/*
 * ComplementaryFilter.h
 *
 *  Created on: 23.08.2015
 *      Author: andy
 */

#ifndef COMPLEMENTARYFILTER_H_
#define COMPLEMENTARYFILTER_H_

class ComplementaryFilter {
public:
	ComplementaryFilter();
	virtual ~ComplementaryFilter();
	void updateValue(double accelerometerAngle, double gyroRate);
	double getAngle();
	double getDt();
	void setT(double t);
private:
	// T defines the averaging period for accelerometerAngle
	// bigger T increases the use of the gyroscope over the accelerometer
	// smaller T increases the use of the accelerometer over the gyroscope
	// use Arduino Serial Plotter over accelerometerAngle and angle to tune T
	double T; // in seconds

	unsigned long lastTime;
	double angle;
	double dt;
	void calculateDt();
};

#endif /* COMPLEMENTARYFILTER_H_ */
