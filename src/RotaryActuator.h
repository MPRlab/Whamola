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
#include <QEIx4.h>
#include <dsp.h>

enum State{
	STATE_IDLE_COAST, 
	STATE_POSITION_CONTROL, // TODO: add current and velocity PID (maybe this is just a position PID) 
	STATE_VELOCITY_CONTROL,
	STATE_CURRENT_CONTROL,
	STATE_COAST_STRIKE,
	STATE_IDLE
};



class RotaryActuator
{
public:
	// CONSTRUCTORS - pass these arguments in as pointers to classes (i.e. "new QEI(PD_1, PD_0, NC, STRIKER_ENCODER_RESOLUTION)")
	RotaryActuator(QEIx4 * Encoder, SingleMC33926MotorController * Motor, int controlInterval_ms, float pKp, float pKi, float pKd);
	RotaryActuator(QEIx4 * Encoder, SingleMC33926MotorController * Motor, int controlInterval_ms, float pKp, float pKi, float pKd, float vKp, float vKi, float vKd);
	RotaryActuator(QEIx4 * Encoder, SingleMC33926MotorController * Motor, int controlInterval_ms);

	// DESTRUCTOR
	~RotaryActuator();

	// Public Methods
	void calibrate(int homePosDist); // zeroes out the encoder at the string and moves to a given home position off of the string 

	// Position Control state methods
	void setPosSetpoint(int encoderSetpoint); 
	void setVelSetpoint(float encoderVelSetpoint); // in encoder ticks / control loop
	void goHome(); // goes to home position
	void goToString(); // goes to the calibrated zero position, temporary dampener
	void setHomePos(int encoderTicks);
	void setCurrentOffsetThreshold(float currentOffsetThresholdInput);


	// Basic Getters and setters
	float readPos();
	void resetEncoder();
	void driveMotor(float power);
	// (still figruing this out) For going between holding a position using PID and executing a gesture (maybe a trajectory or simply an impulse drive coast)

	// void setState(State newState);
	// Velocity control methods:

	// sends motor coast-driving at a duty cycle and releases at a certain distance off string
	// TODO: have it take in a linear velocity and use linear distance instead of encoder ticks
	void coastStrike(float encVel, int releaseDistance, int timeWaitAfterStrike); // release distance in ticks for now

	void dampenString(bool hard); // TODO: dampen string hard or soft by controlling for current 
	void controlLoop(); // runs upon ticker firing AFTER actuator is calibrated
	
	State _state = STATE_IDLE;

private:

	// Private methods
	float clampToMotorVal(float output); // used internally to condition PID output


	// Private attributes
	Ticker _tick;
	bool _isCalibrated = false;
	int _posSetpoint;
	float _pos; // *** Might need to change this to a float
	float _velSetpoint;
	int _homePos;

	int _controlInterval_ms;
	uint32_t _controlLoopCounter = 0;
	int _lastPos = 0;
	float _currentOffsetThreshold = 0.025;

	// Coast-Strike state variables
	bool _letGo = false;
	float _motorPower;
	int _stopAtCount, _releaseDistance;

	// Code taken from the mbed eventqueue API reference page:
	// Create a queue that can hold a maximum of 32 events
	// EventQueue queue;
	// Create a thread that'll run the event queue's dispatch function
	// Thread t;

	// CMSIS DSP ARM PID classes 
	arm_pid_instance_f32 * _ArmPosPid;
	arm_pid_instance_f32 * _ArmVelPid;

	// Passed in member classes
	QEIx4 * _Encoder;
	SingleMC33926MotorController * _Motor;

};

#endif