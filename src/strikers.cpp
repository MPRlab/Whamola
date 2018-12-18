#include "mbed.h"
#include "DualMC33926MotorShield.h"
#include "QEI.h"

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


int main(void){
	pc.puts("Enabling\n");
	Dampener.enable();
	pc.puts("Sending a speed command, press user button to stop\n");
	while(!stopButton){
		Dampener.setM1Speed(0.9f); // Set speed between -1.0 to 1.0
		pc.printf("current reading: %f Amps\tfault status: %d \n", Dampener.getM1CurrentAmps(), Dampener.hasFault());

		//M1Pwm->write(0.5f);
		wait(0.25);
	}
	pc.puts("stopping\n");
	Dampener.setM1Speed(0.0f);
	return 0;
}