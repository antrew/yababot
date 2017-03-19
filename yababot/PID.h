#ifndef PID_H
#define PID_H

class PID {
public:
	PID(double P, double I, double D);
	double perform(double error, double dt);
	void setCoefficients(double P, double I, double D);
private:
	double P;
	double I;
	double D;
	double integralError;
	double lastError;
};

#endif // PID_H
