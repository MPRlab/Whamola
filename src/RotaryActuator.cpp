/*
 * This class serves for all the DC brushed motor and encoder 
 * combo actuators in the system (the Dampener and the Strikers)
 *
 * Author: Sean O'Neil
 *
 */

#include "RotaryActuator.h"

RotaryActuator::RotaryActuator(QEI * Encoder, SingleMC33926MotorController * Motor, 
								float controlInterval, float pKp, float pKi, float pKd,
								float vKp, float vKi, float vKd)
{
	_Encoder = Encoder;
	_Motor = Motor;
	_ArmPosPid = new arm_pid_instance_f32;
	_ArmVelPid = new arm_pid_instance_f32;

	_tick.attach(this, &RotaryActuator::controlLoop, controlInterval);

	// Set PID Parameters following this forum post (https://os.mbed.com/questions/1904/mbed-DSP-Library-PID-Controller/)
	_ArmPosPid->Kp = pKp;
	_ArmPosPid->Ki = pKi;
	_ArmPosPid->Kd = pKd;
	arm_pid_init_f32(this->_ArmPosPid, true); // Passes in DampPID_cmsis struct and set reset flag to true

	_ArmVelPid->Kp = vKp;
	_ArmVelPid->Ki = vKi;
	_ArmVelPid->Kd = vKd;
	arm_pid_init_f32(this->_ArmVelPid, true);

}

// This constructor uses default values for the velocity PID constants
RotaryActuator::RotaryActuator(QEI * Encoder, SingleMC33926MotorController * Motor, 
								float controlInterval, float pKp, float pKi, float pKd)
{
	_Encoder = Encoder;
	_Motor = Motor;
	_ArmPosPid = new arm_pid_instance_f32;
	_ArmVelPid = new arm_pid_instance_f32;

	_tick.attach(this, &RotaryActuator::controlLoop, controlInterval);

	// Set PID Parameters following this forum post (https://os.mbed.com/questions/1904/mbed-DSP-Library-PID-Controller/)
	_ArmPosPid->Kp = pKp;
	_ArmPosPid->Ki = pKi;
	_ArmPosPid->Kd = pKd;
	arm_pid_init_f32(this->_ArmPosPid, true); // Passes in DampPID_cmsis struct and set reset flag to true

	_ArmVelPid->Kp = 0.001;
	_ArmVelPid->Ki = 0.0;
	_ArmVelPid->Kd = 0.0;
	arm_pid_init_f32(this->_ArmVelPid, true);
}

RotaryActuator::~RotaryActuator(){
	delete _Encoder;
	delete _Motor;
	delete _ArmPosPid;
	delete _ArmVelPid;
}

void RotaryActuator::calibrate(int homePosDist){
	float amps;
	float avgCurrent = 0.0f;
	float currentSum = 0.0f;		
	int ticks = 0;
	int currentCount = 0;

	_Motor->enable();
	Timer t;

	// Start driving the motor towards the string
	_Motor->setSpeedCoast(-0.1f); // Set speed between -1.0 to 1.0
	t.start();

	// Take in some measurements as a baseline
	while(t.read() < 0.2){
		amps = _Motor->getCurrent();
		currentSum += amps;
		currentCount++;
	}


	avgCurrent = currentSum / currentCount;
	float currentThreshold = avgCurrent + 0.06;
	printf("current to detect zero point on the string: %f A\n", currentThreshold);
	currentSum = 0.0;
	currentCount = 0;

	// Drive until the string is sensed, i.e. current exceeds threshold
	t.reset();
	while(avgCurrent < currentThreshold){
		amps = _Motor->getCurrent();
		currentSum += amps;
		currentCount++;

		if(t.read() > 0.25){
			t.reset();
			avgCurrent = currentSum / currentCount;
			printf("avgCurrent: %f A\n", avgCurrent);
			currentSum = 0;
			currentCount = 0;
		}

	}
	// printf("avgCurrent: %f A\n", avgCurrent);

	//pc.puts("stopping\n");
	_Encoder->reset();
	_Motor->setSpeedBrake(0.0f);

	// sets home position to be slightly above where the string is
	setHomePos(_Encoder->getPulses() - homePosDist);
	setPosSetpoint(_homePos);

	_state = STATE_POSITION_CONTROL;
}

void RotaryActuator::setPosSetpoint(int encoderSetpoint){
	_posSetpoint = encoderSetpoint;
}

void RotaryActuator::setVelSetpoint(float encoderVelSetpoint){ // in encoder ticks / second
	_velSetpoint = encoderVelSetpoint;
}

void RotaryActuator::setHomePos(int encoderTicks){
	_homePos = encoderTicks;
}

void RotaryActuator::goToString(){
	setPosSetpoint(0);
}

float RotaryActuator::clampToMotorVal(float output){
	if(output > 1.0)
		output = 1.0;
	else if(output < -1.0)
		output = -1.0;
	return output;
}


// This method is called periodically on the ticker object
void RotaryActuator::controlLoop(){
	
	float pos = _Encoder->getPulses();


	if(_state == STATE_POSITION_CONTROL){
		float out = arm_pid_f32(this->_ArmPosPid, _posSetpoint - pos);
		out = - clampToMotorVal(out); // Negative here cuz the motor expects it the other way
		printf("PID output: %f\tPosition: %f\tSetpoint: %d\n", out, pos, _posSetpoint);
		_Motor->setSpeedBrake(out); // TODO: see about changing this to a coast drive

	}
	else if(_state == STATE_VELOCITY_CONTROL){
		float vel = pos - _lastPos;
		float out = arm_pid_f32(this->_ArmVelPid, _velSetpoint - vel);
		out = - clampToMotorVal(out);
		_Motor->setSpeedCoast(out); // This uses coast for the bounceback effect on the string for gestures

	}

	else{ // _state == STATE_CURRENT_CONTROL
		float current = _Motor->getCurrent(); // TODO: add current control mode
	}

	_lastPos = pos;
	_controlLoopCounter++;
}