#include "mbed.h"
#include "DualMC33926MotorShield.h"
#include <QEI.h>
#include <PID.h>

#define STRIKER_ENCODER_RESOLUTION 3200

// Function prototypes
void zeroDampenerOnString(void);
void moveOneRotation(void);
void goToPosition(int setpoint, float interval);

// User button for shutoff
DigitalIn stopButton(USER_BUTTON);

// Communication
Serial pc(USBTX, USBRX);

// Timing
Timer t;
float loopTime = 0.01;

// Init dampener motor, encoders, and PID
DualMC33926MotorShield Dampener(D11, D15, A0, D3, D5);
QEI DampEnc(PD_1, PD_0, NC, STRIKER_ENCODER_RESOLUTION);
PID DampPID(1.0f, 0.0f, 0.0f, loopTime); // Kp, Ki, Kd, interval


// DualMC33926MotorShield RightStriker(D8,D14);
// DualMC33926MotorShield LeftStriker(D7,D13);


//PwmOut M1Pwm(D15);
//PwmOut * M1Pwm = new PwmOut(D15);


int main(void){
	
	// Set PID Parameters
	DampPID.setInputLimits(-3200.0, 0.0); // Use encoder ticks as input
	DampPID.setOutputLimits(-1.0, 1.0); // Use half power of motor as output limits for now

	while(!stopButton); // stay until I tell you to

	zeroDampenerOnString();
	wait(1);
	pc.puts("Moving to a resting position above the string\n\n");
	goToPosition(-300, loopTime);
	return 0;
}

void goToPosition(int setpoint, float interval){ // setpoint in ticks, set to -200 for a decent resting place above string
	int posError = setpoint - DampEnc.getPulses();
	int position;
	float pidSpeed;
	while(!stopButton){// && abs(setpoint - position) > 10){ // stop when within a certain tolerance area
		t.reset();
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
	Dampener.setM1Speed(-0.1f); // Set speed between -1.0 to 1.0
	t.start();

	while(t.read() < 0.25){
		amps = Dampener.getM1CurrentAmps();
		currentSum += amps;
		currentCount++;
	}

	avgCurrent = currentSum / currentCount;
	float currentThreshold = avgCurrent + 0.05;
	printf("current to detect zero point on the string: %f A\n", currentThreshold);
	currentSum = 0.0;
	currentCount = 0;

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