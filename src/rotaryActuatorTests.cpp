#include <mbed.h>
#include "RotaryActuator.h"
// #include "SingleMC33926MotorController.h"
// #include <QEI.h>

#define STRIKER_ENCODER_RESOLUTION 3200

// Init variables
int setpoint = -100;
float loopTime = 0.05f;


// Communication
Serial pc(USBTX, USBRX);

// User button for shutoff
DigitalIn stopButton(USER_BUTTON);


// Dampener Actuator Init
RotaryActuator Dampener(new QEI(PD_1, PD_0, NC, STRIKER_ENCODER_RESOLUTION),
						new SingleMC33926MotorController(D11, D15, A0, D3, D5, false), 
						loopTime, 0.005f, 0.0f, 0.0f);

// Right striker Actuator Init
RotaryActuator StrikerR(new QEI(PD_4, PD_3, NC, STRIKER_ENCODER_RESOLUTION),
						new SingleMC33926MotorController(D8, D14, A2, PB_10, PE_15, true), 
						loopTime, 0.005f, 0.0f, 0.0f);

// Left striker Actuator Init
RotaryActuator StrikerL(new QEI(PD_6, PD_5, NC, STRIKER_ENCODER_RESOLUTION),
						new SingleMC33926MotorController(D7, D13, A1, PE_14, PB_11, true), 
						loopTime, 0.005f, 0.0f, 0.0f);


int main(){
	

	while(!stopButton); // stay until I tell you to

	// pc.puts("calibrating dampener now...\n");
	// Dampener.calibrate(100);
	// pc.puts("Done calibrating dampener\n");

	pc.puts("calibrating right striker now...\n"); // TODO: Figure out why right striker encoder is not being read
	StrikerR.calibrate(100);
	pc.puts("Done calibrating right striker now\n");

	// pc.puts("calibrating left striker now...\n");
	// StrikerL.calibrate(100);
	// pc.puts("Done calibrating left striker\n");



	while(!stopButton); // stay until I tell you to


	return 0;
}