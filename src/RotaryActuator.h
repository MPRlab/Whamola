/*
 * This class serves for all the DC brushed motor and encoder 
 * combo actuators in the system (the Dampener and the Strikers)
 *
 * Author: Sean O'Neil
 *
 */

#ifndef RotaryActuator_h
#define RotaryActuator_h

#include "mbed.h"
#include "SingleMC33926MotorController.h"
#include <QEI.h>
#include <dsp.h>


class RotaryActuator
{
public:
	// CONSTRUCTOR - pass these arguments in as pointers to classes (i.e. "new QEI(PD_1, PD_0, NC, STRIKER_ENCODER_RESOLUTION)")
	RotaryActuator(arm_pid_instance_f32 * ArmPid, QEI * Encoder, SingleMC33926MotorController * Motor, float controlInterval);

	// DESTRUCTOR
	~RotaryActuator();

	// Public Methods
	void calibrate(int homePos); // zeroes out the encoder at the string and moves to a given home position off of the string 

	// PID state methods
	void setSetpoint(int encoderSetpoint); 
	void goHome(); // goes to home position
	void goToString(); // goes to the calibrated zero position, temporary dampener



	// GESTURE state methods:

	// sends motor coast-driving at a duty cycle and releases at a certain distance off string
	// TODO: have it take in a rotational velocity in rev/sec and calculate appropriate Duty Cycle OR control for it
	void coastStrike(float pwmSpeed, int releaseDistance); 
	void dampenString(bool hard); // dampen string hard or soft by controlling for current 


private:

	// Private methods
	void controlLoop(); // runs upon ticker firing AFTER actuator is calibrated
	float clampToMotorVal(float output); // used internally to condition PID output


	// Private attributes
	Ticker tick;
	bool isCalibrated = false;

	// (still figruing this out) For going between holding a position using PID and executing a gesture (maybe a trajectory or simply an impulse drive coast)
	enum _State{
		STATE_PID, // TODO: add current and velocity PID (maybe this is just a position PID) 
		STATE_GESTURE
	};

	int setpoint;

	// Passed in member classes
	arm_pid_instance_f32 * ArmPid;
	QEI * _Encoder;
	SingleMC33926MotorController * _Motor;

};