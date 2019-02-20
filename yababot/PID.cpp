/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2017  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "PID.h"

PID::PID(double P, double I, double D) {
	this->P = P;
	this->I = I;
	this->D = D;
	this->lastError = 0;
	this->integralError = 0;
}

double PID::perform(double error, double dt) {
	integralError += error * dt;
	float differentialError = (error - lastError) / dt;
	lastError = error;

	float up = P * error;
	float ui = I * integralError;
	float ud = D * differentialError;
	float u = up + ui + ud;

	return u;
}

void PID::setCoefficients(double P, double I, double D) {
	this->P = P;
	this->I = I;
	this->D = D;
}
