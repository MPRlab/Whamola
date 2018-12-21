/*
	Messy Test Code for actuating the strikers without the RotaryActuator Library
	Author: Sean O'Neil
*/
#include "mbed.h"
#include "DualMC33926MotorShield.h"
#include "SingleMC33926MotorController.h"
#include <QEI.h>
#include <PID.h>
#include <dsp.h>


#define STRIKER_ENCODER_RESOLUTION 3200

// Function prototypes
void zeroDampenerOnString(void);
void moveOneRotation(void);
void goToPosition(int setpoint, float interval);
void cmsisControlLoop();

// User button for shutoff
DigitalIn stopButton(USER_BUTTON);

// Communication
Serial pc(USBTX, USBRX);

// Timing
Timer t;
Ticker ticker;
float loopTime = 0.05;

// Init dampener actuator
DualMC33926MotorShield Dampener(D11, D15, A0, D3, D5);
QEI DampEnc(PD_1, PD_0, NC, STRIKER_ENCODER_RESOLUTION);
arm_pid_instance_f32 DampPID_cmsis;
int setpoint = -100;
PID DampPID(1.0f, 0.0f, 0.0f, loopTime); // Kp, Ki, Kd, interval

// Init Striker left actuator
SingleMC33926MotorController StrikerL(D7, D13, A1, PE_14, PB_11);

// Init Striker right actuator
SingleMC33926MotorController StrikerR(D8, D14, A2, PB_10, PE_15);

// DualMC33926MotorShield RightStriker(D8,D14);
// DualMC33926MotorShield LeftStriker(D7,D13);


// Testing
DigitalOut testDir(PG_0);

int main(void){

	// Test
	testDir.write(1);
	wait(5);
	testDir.write(0);
	
	
	// Set PID Parameters following this forum post (https://os.mbed.com/questions/1904/mbed-DSP-Library-PID-Controller/)
	DampPID_cmsis.Kp = 0.005;
	DampPID_cmsis.Ki = 0.0;
	DampPID_cmsis.Kd = 0.0;
	arm_pid_init_f32(&DampPID_cmsis, 1); // Passes in DampPID_cmsis struct and set reset flag to true

	/*
	DampPID.setInputLimits(-3200.0, 0.0); // Use encoder ticks as input
	DampPID.setOutputLimits(-1.0, 1.0); // Use half power of motor as output limits for now
	*/
	while(!stopButton); // stay until I tell you to
/*
	// Debugging stuff 
	StrikerL.enable();
	StrikerR.enable();

	StrikerL.setSpeedBrake(0.7f);
	StrikerR.setSpeedBrake(0.7f);
	wait(10);
	StrikerL.setSpeedBrake(-0.3f);
	StrikerR.setSpeedBrake(-0.3f);
	wait(10);
	StrikerL.setSpeedBrake(0.0f);
	StrikerR.setSpeedBrake(0.0f);
*/

	zeroDampenerOnString();
	wait(1);
	ticker.attach(&cmsisControlLoop, loopTime);


	StrikerL.setSpeedCoast(1.0f);
	StrikerR.setSpeedCoast(1.0f);
	wait(0.2);
	StrikerL.setSpeedCoast(0.0f);
	StrikerR.setSpeedCoast(0.0f);
	

	// pc.puts("Moving to a resting position above the string\n\n");
	// goToPosition(-300, loopTime);
	while(!stopButton){
		printf("Right Motor Current: %f\tLeft Motor Current: %f\n", StrikerR.getCurrent(), StrikerL.getCurrent());
	}
	StrikerL.setSpeedBrake(0.0f);
	StrikerR.setSpeedBrake(0.0f);
	ticker.detach();
	Dampener.setM1Speed(0.0f);

	return 0;
}

float clampToMotorVal(float output){
	if(output > 1.0)
		output = 1.0;
	else if(output < -1.0)
		output = -1.0;
	return output;
}

void cmsisControlLoop(){
	int pos = DampEnc.getPulses();
	float out = arm_pid_f32(&DampPID_cmsis, setpoint - pos);
	out = - clampToMotorVal(out); // Negative here cuz the motor expects it the other way
	//printf("PID output: %f\tPosition: %d\n", out, pos);
	Dampener.setM1Speed(out);

}

//Function based on old PID.h library
void goToPosition(int setpoint, float interval){ // setpoint in ticks, set to -200 for a decent resting place above string
	int posError = setpoint - DampEnc.getPulses();
	int position;
	float pidSpeed;
	while(!stopButton){// && abs(setpoint - position) > 10){ // stop when within a certain tolerance area
		t.reset();
		posError = setpoint - DampEnc.getPulses();
		position =  DampEnc.getPulses();
		DampPID.setProcessValue(position);
		pidSpeed = DampPID.compute();
		// Dampener.setM1Speed(-pidSpeed);  // TAKE A HARD LOOK AT THIS NEGATIVE SIGN OR ELSE IT WILL GOT THE OTHER WAY 
		printf("Position: %d\tPID speed: %f\tMy calculated P speed: %f\n", position, pidSpeed, 1.0f * (setpoint - position));

		while(t.read() < interval); // waits out the rest of the time interval
	}
	pc.puts("exited the loop, setting speed to zero");
	Dampener.setM1Speed(0.0f);

}

void zeroDampenerOnString(){
	float amps, revs;
	float avgCurrent = 0.0f;
	float currentSum = 0.0f;		
	int ticks = 0;
	int currentCount = 0;
	pc.puts("Enabling\n");
	Dampener.enable();

	// Start driving the motor towards the string
	Dampener.setM1Speed(-0.1f); // Set speed between -1.0 to 1.0
	t.start();

	// Take in some measurements as a baseline
	while(t.read() < 0.2){
		amps = Dampener.getM1CurrentAmps();
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
	while(!stopButton && avgCurrent < currentThreshold){
		amps = Dampener.getM1CurrentAmps();
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
	printf("avgCurrent: %f A\n", avgCurrent);

	pc.puts("stopping\n");
	Dampener.setM1Speed(0.0f);
	DampEnc.reset();
}

void moveOneRotation(){
	float amps, revs;		
	int ticks;
	pc.puts("Enabling\n");
	Dampener.enable();
	pc.puts("Sending a speed command, press user button to stop\n");
	Dampener.setM1Speed(0.7); // Set speed between -1.0 to 1.0

	t.start();
	while(!stopButton && abs(revs) < 1.0){
		ticks = DampEnc.getPulses();
		revs = (float) (ticks / STRIKER_ENCODER_RESOLUTION);
		amps = Dampener.getM1CurrentAmps();

		if(t.read() > 0.4){
			pc.printf("current reading: %f Amps\tfault status: %d \t Position: %d ticks\n", amps, Dampener.hasFault(), ticks);
			t.reset();
		}
		//M1Pwm->write(0.5f);
	}
	pc.puts("stopping\n");
	Dampener.setM1Speed(0.0f);
	wait(5);
}