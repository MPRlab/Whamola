#include "mbed.h"
#include "DualMC33926MotorShield.h"

// User button for shutoff
DigitalIn stopButton(USER_BUTTON);

Serial pc(USBTX, USBRX);

// Init striker motors
DualMC33926MotorShield Dampener(D11, D15, A0, D3, D5);
// DualMC33926MotorShield RightStriker(D8,D14);
// DualMC33926MotorShield LeftStriker(D7,D13);

// Test
//PwmOut M1Pwm(D15);
//PwmOut * M1Pwm = new PwmOut(D15);


int main(){
	pc.puts("Enabling\n");
	Dampener.enable();
	wait(2);
	pc.puts("Sending a speed command\n");
	Dampener.setM1Speed(300.0); // Set speed between -400.0 and 400.0
	//M1Pwm->write(0.5f);
	wait(2);
	pc.puts("going into an infinite while loop\n");
	while(1);
	return 0;
}