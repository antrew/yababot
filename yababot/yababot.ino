/*
 * Getting Started example sketch for nRF24L01+ radios
 * This is a very basic example of how to send data from one node to another
 * Updated: Dec 2014 by TMRh20
 */

#include <SPI.h>
#include <printf.h>
#include "RF24.h"
#include "../common/common.h"
#include "Motor.h"

#include <MPU6050.h>

#include <math.h>
#include "ComplementaryFilter.h"
#include "PID.h"

MPU6050 mpu;
ComplementaryFilter complementaryFilter;

// 2 and 3 are the only two pins supporting hardware interrupts
const uint8_t LEFT_ENCODER_INTERRUPT_PIN = 2;
const uint8_t RIGHT_ENCODER_INTERRUPT_PIN = 3;
const uint8_t LEFT_ENCODER_SECOND_PIN = 4;
const uint8_t RIGHT_ENCODER_SECOND_PIN = 5;

// motor enable pins must be PWM
const uint8_t MOTOR_LEFT_ENABLE = 9;
const uint8_t MOTOR_LEFT_FORWARD = 8;
const uint8_t MOTOR_LEFT_BACKWARD = 7;

// motor enable pins must be PWM
const uint8_t MOTOR_RIGHT_ENABLE = 10;
const uint8_t MOTOR_RIGHT_FORWARD = A3;
const uint8_t MOTOR_RIGHT_BACKWARD = A2;

const uint8_t RADIO_CE_PIN = A0;
const uint8_t RADIO_CS_PIN = A1;
// RADIO MOSI = 11
// RADIO MISO = 12
// RADIO SCK  = 13

const char * FW_ID = "yababot";

const double PID_P = 0;
const double PID_I = 0;
const double PID_D = 0;

const double ROTATION_PID_P = 0.01;

const uint8_t CALIBRATION_TIME_SECONDS = 3;

/**
 * Critical angle that cannot be normally recovered.
 * Motors will be switched off when this angle is reached.
 */
const double FAILSAFE_CRITICAL_ANGLE = 70;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(RADIO_CE_PIN, RADIO_CS_PIN);
/**********************************************************/

Motor leftMotor = Motor(MOTOR_LEFT_ENABLE, MOTOR_LEFT_BACKWARD, MOTOR_LEFT_FORWARD);
Motor rightMotor = Motor(MOTOR_RIGHT_ENABLE, MOTOR_RIGHT_BACKWARD, MOTOR_RIGHT_FORWARD);

bool motorsEnabled = false;

PID pid = PID(PID_P, PID_I, PID_D);

PID rotationPid = PID(ROTATION_PID_P, 0, 0);

double setPoint = 0;

int8_t joystickForward = 0;
int8_t joystickRotate = 0;

void setupSensors() {
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
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

	radio.openReadingPipe(1, RADIO_ADDRESS);

	// Start the radio listening for data
	radio.startListening();

	radio.printDetails();

	setupSensors();

	motorsEnabled = false;
	leftMotor.off();
	rightMotor.off();
}

void processSensors() {
  Vector rawAccel = mpu.readRawAccel();

//  Serial.print(" Xraw = ");
//  Serial.print(rawAccel.XAxis);
//  Serial.print(" Yraw = ");
//  Serial.print(rawAccel.YAxis);
//  Serial.print(" Zraw = ");
//  Serial.print(rawAccel.ZAxis);

  // Y points forward
  // X points left
  // Z points down
  // convert atan2 from radians to grads
  double accelAngle = 180 / M_PI * atan2( -rawAccel.YAxis, -rawAccel.ZAxis);

  // gyroscope is already in grads
  Vector normGyro = mpu.readNormalizeGyro();
  double gyroRate = normGyro.XAxis;
  
//  Serial.print(" accelAngle = ");
//  Serial.print(accelAngle);
//  Serial.print(" gyroRate = ");
//  Serial.print(gyroRate);
//  Serial.println();
  
  complementaryFilter.updateValue(accelAngle, gyroRate);
}

void control() {
  // turn off motors if the angle is more than critical
  if (abs(complementaryFilter.getAngle()) > FAILSAFE_CRITICAL_ANGLE) {
    Serial.println("Critical angle detected. Switching motors off.");
    leftMotor.off();
    rightMotor.off();
  }

  double error = setPoint - complementaryFilter.getAngle();
  double u = pid.perform(error, complementaryFilter.getDt());

  double rotationU = rotationPid.perform(joystickRotate, complementaryFilter.getDt());

  leftMotor.setDirection(u + rotationU);
  rightMotor.setDirection(u - rotationU);
}

void toggleMotors() {
	Serial.print("Toggling motors ");
	if (motorsEnabled) {
		Serial.println("off");
		motorsEnabled = false;
		leftMotor.off();
		rightMotor.off();
	} else {
		Serial.println("on");
		motorsEnabled = true;
		leftMotor.on();
		rightMotor.on();
	}
}

void calibrate() {
	Serial.println("Begin calibration");

	// calibrate gyro
	mpu.calibrateGyro();

	// calibrate setPoint
	unsigned long endCalibrationMillis = millis() + CALIBRATION_TIME_SECONDS * 1000;
	unsigned long iterations = 0;
	double sumAngle = 0;
	while (millis() < endCalibrationMillis) {
		processSensors();
		sumAngle += complementaryFilter.getAngle();
		iterations++;
	}
	setPoint = sumAngle / iterations;
	Serial.print("End calibration. setPoint = ");
	Serial.print(setPoint);
	Serial.print(" iterations = ");
	Serial.print(iterations);
	Serial.println();
}

void processRadio() {
	if (radio.available()) {
		struct radioMessage message;
		while (radio.available()) {             // While there is data ready
			radio.read(&message, sizeof(message)); // Get the payload

			switch (message.command) {
				case TOGGLE_MOTORS:
					toggleMotors();
					break;
				case CALIBRATE:
					calibrate();
					break;
				case SET_PID_COEFFICIENTS:
					pid.setCoefficients(message.pidCoefficients.pidP, message.pidCoefficients.pidI, message.pidCoefficients.pidD);
					break;
				case CONTROL:
					joystickForward = message.control.forward;
					joystickRotate = message.control.rotate;
					break;
			}
		}
	}
}

void loop() {
	// read MPU6050 here
	processSensors();
	// balance
	control();
	// process radio commands
	processRadio();
} // Loop
