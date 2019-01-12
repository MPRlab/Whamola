/*
 * This class serves for all the DC brushed motor and encoder 
 * combo actuators in the system (the Dampener and the Strikers)
 *
 * Author: Sean O'Neil
 *
 */

#include "RotaryActuator.h"

RotaryActuator::RotaryActuator(QEIx4 * Encoder, SingleMC33926MotorController * Motor, 
								int controlInterval_ms, float pKp, float pKi, float pKd,
								float vKp, float vKi, float vKd)
{
	_Encoder = Encoder;
	_Motor = Motor;
	_ArmPosPid = new arm_pid_instance_f32;
	_ArmVelPid = new arm_pid_instance_f32;

	// Code taken from the mbed eventqueue API reference page:

	// Create a queue that can hold a maximum of 32 events
	// EventQueue queue(32 * EVENTS_EVENT_SIZE);
	// Create a thread that'll run the event queue's dispatch function
	// Thread t;

	// Start the event queue's dispatch thread
	// t.start(callback(&queue, &EventQueue::dispatch_forever));

	_controlInterval_ms = controlInterval_ms;
	// _tick.attach(this, &RotaryActuator::controlLoop, controlInterval);
	// queue.call_every(controlInterval_ms, this, &RotaryActuator::controlLoop);


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
RotaryActuator::RotaryActuator(QEIx4 * Encoder, SingleMC33926MotorController * Motor, 
								int controlInterval_ms, float pKp, float pKi, float pKd)
{
	_Encoder = Encoder;
	_Motor = Motor;
	_ArmPosPid = new arm_pid_instance_f32;
	_ArmVelPid = new arm_pid_instance_f32;

	// Code taken from the mbed eventqueue API reference page:

	// Create a queue that can hold a maximum of 32 events
	// EventQueue queue(32 * EVENTS_EVENT_SIZE);
	// Create a thread that'll run the event queue's dispatch function
	// Thread t;

	// Start the event queue's dispatch thread
	// t.start(callback(&queue, &EventQueue::dispatch_forever));

	_controlInterval_ms = controlInterval_ms;
	// _tick.attach(this, &RotaryActuator::controlLoop, controlInterval);
	// queue.call_every(controlInterval_ms, this, &RotaryActuator::controlLoop);


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

	// Start driving the motor away from the string
	_Motor->setSpeedBrake(-0.15f); 
	wait(1);
	_Motor->setSpeedBrake(0.2f); 
	t.start();

	// Take in some measurements as a baseline
	while(t.read() < 0.25){
		amps = _Motor->getCurrent();
		currentSum += amps;
		currentCount++;
	}


	avgCurrent = currentSum / currentCount;
	float currentThreshold = avgCurrent + 0.025;
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
	_Motor->setSpeedCoast(0.0f);
	wait(0.25);
	_Encoder->write(0);


	// sets home position to be slightly above where the string is
	setHomePos(_Encoder->getPosition() - homePosDist);

	setPosSetpoint(_Encoder->getPosition() - homePosDist);
	
	_state = STATE_POSITION_CONTROL;
	printf("encoder reading: %f, home position at %d changing state to position control...\n", _Encoder->getPosition(), _homePos);
}

void RotaryActuator::setCurrentOffsetThreshold(float currentOffsetThresholdInput){
	_currentOffsetThreshold = currentOffsetThresholdInput;
}

void RotaryActuator::setPosSetpoint(int encoderSetpoint){
	_posSetpoint = encoderSetpoint;
}

void RotaryActuator::setVelSetpoint(float encoderVelSetpoint){ // in encoder ticks / second
	// TODO: Implement velocity control
}

void RotaryActuator::setHomePos(int encoderTicks){
	_homePos = encoderTicks;
}

void RotaryActuator::goToString(){
	setPosSetpoint(0);
}

void RotaryActuator::goHome(){
	setPosSetpoint(_homePos);
}

float RotaryActuator::readPos(){
	return _Encoder->getPosition();
}
void RotaryActuator::resetEncoder(){
	_Encoder->write(0);
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
	// Timer t;
	// t.reset();
	_pos = _Encoder->getPosition();
	float current = _Motor->getCurrent();

	if(_state == STATE_IDLE_COAST){
		_Motor->setSpeedCoast(0.0f);
		printf("Encoder ticks: %f\n", _Encoder->getPosition());
	}
	else if(_state == STATE_POSITION_CONTROL){
		float error = (float) _posSetpoint - _pos;
		float out = arm_pid_f32(this->_ArmPosPid, error);


		// float out = error * 0.005f;

		// if(!_Motor->isFlipped())
		// 	out = - clampToMotorVal(out); 
		// else
		// 	out = clampToMotorVal(out); 

		// printf("PID output: %f\tPosition: %f\tSetpoint: %d\tError: %f\n", out, _Encoder->getPosition(), _posSetpoint, error);
		_Motor->setSpeedBrake(out); // TODO: see about changing this to a coast drive if it needs it

	}
	else if(_state == STATE_VELOCITY_CONTROL){
		float vel = _pos - _lastPos;
		float out = arm_pid_f32(this->_ArmVelPid, _velSetpoint - vel);
		out = clampToMotorVal(out);
		_Motor->setSpeedCoast(out); // This uses coast for the bounceback effect on the string for gestures

	}
	else if(_state == STATE_CURRENT_CONTROL){
		// TODO: add current control mode
	}
	else if(_state == STATE_COAST_STRIKE){
		if(_letGo){
			_Motor->setSpeedCoast(0.0f);
			// _Motor->disable();
			if(_controlLoopCounter > _stopAtCount){ // 100 loops = 1 second
				// _Motor->setSpeedBrake(0.0f);

				_state = STATE_POSITION_CONTROL;
				printf("Encoder now at %d, switching to STATE_POSITION_CONTROL\n", _pos);

			}
		}
		else{
			_Motor->setSpeedCoast(_motorPower);
			printf("Encoder ticks: %f\n", _Encoder->getPosition());
			if(_Encoder->getPosition() > - abs(_releaseDistance)){
				_letGo = true;
				_controlLoopCounter = 0;
			}
		}
		// printf("Motor current: %f\tEncoder: %f\n", current, _Encoder->getPosition());
	}
	else{ // _state == STATE_IDLE
		//printf("Encoder ticks: %d\n", _Encoder->getPosition()); // TODO: change this to some default thing or something that does nothing
	} 

	_lastPos = _pos;
	_controlLoopCounter++;

	// printf("Execution time per loop: %f\n",t.read());
}

// TODO: make this function non-blocking, i.e. make the control loop follow a trajectory
void RotaryActuator::coastStrike(float encVel, int releaseDistance, int timeWaitAfterStrike_ms){

	_letGo = false;
	_motorPower = encVel; // solution for now
	_releaseDistance = releaseDistance;
	_stopAtCount = timeWaitAfterStrike_ms / _controlInterval_ms;
	if(_Encoder->getPosition() > - abs(_releaseDistance)){
		printf("release point %d is currently greater than striker location, will not carry out coastStrike\n", _stopAtCount);
	}
	else{
		_controlLoopCounter = 0;
		_state = STATE_COAST_STRIKE;
	}
}


void RotaryActuator::dampenString(EventQueue * queue, bool hard, int time_ms){
	//TODO: implement a hard vs. soft dampening
	// _stopAtCount = time_ms / _controlInterval_ms;
	// _controlLoopCounter = 0;
	_state = STATE_POSITION_CONTROL;
	// goToString();
	setPosSetpoint(-20);
	queue->call_in(time_ms, this, &RotaryActuator::goHome);

}


void RotaryActuator::driveMotor(float power){
	_Motor->setSpeedBrake(power);
}
