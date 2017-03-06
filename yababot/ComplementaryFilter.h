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
private:
	unsigned long lastTime;
	double angle;
	double dt;
	void calculateDt();
};

#endif /* COMPLEMENTARYFILTER_H_ */
