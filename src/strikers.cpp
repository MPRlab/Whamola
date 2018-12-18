#include "mbed.h"
#include "DualMC33926MotorShield.h"
#include "QEI.h"

#define STRIKER_ENCODER_RESOLUTION 3200

// User button for shutoff
DigitalIn stopButton(USER_BUTTON);

Serial pc(USBTX, USBRX);

// Init striker motors and encoders
DualMC33926MotorShield Dampener(D11, D15, A0, D3, D5);
QEI DampEnc(PD_1, PD_0, NC, STRIKER_ENCODER_RESOLUTION);
// DualMC33926MotorShield RightStriker(D8,D14);
// DualMC33926MotorShield LeftStriker(D7,D13);

// Test
//PwmOut M1Pwm(D15);
//PwmOut * M1Pwm = new PwmOut(D15);


int main(void){
	float amps, revs;		
	int ticks;
	pc.puts("Enabling\n");
	Dampener.enable();
	pc.puts("Sending a speed command, press user button to stop\n");
	Dampener.setM1Speed(0.1); // Set speed between -1.0 to 1.0

	while(!stopButton && abs(revs) < 1.0){
		ticks = DampEnc.getPulses();
		revs = (float) (ticks / STRIKER_ENCODER_RESOLUTION);
		amps = Dampener.getM1CurrentAmps();
		pc.printf("current reading: %f Amps\tfault status: %d \t Position: %d ticks\n", amps, Dampener.hasFault(), ticks);

		//M1Pwm->write(0.5f);
		wait(0.25);
	}
	pc.puts("stopping\n");
	Dampener.setM1Speed(0.0f);
	return 0;
}