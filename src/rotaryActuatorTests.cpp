#include <mbed.h>
#include "RotaryActuator.h"
// #include "SingleMC33926MotorController.h"
// #include <QEI.h>

#define STRIKER_ENCODER_RESOLUTION 3200

// Init variables
int setpoint = -100;
int loopTime = 10; // milliseconds


// Communication
Serial pc(USBTX, USBRX);

// User button for shutoff
DigitalIn stopButton(USER_BUTTON);

// EventQueue eventQueue;
// Thread thread;

// Dampener Actuator Init
// RotaryActuator Dampener(new QEI(PD_1, PD_0, NC, STRIKER_ENCODER_RESOLUTION),
// 						new SingleMC33926MotorController(D11, D15, A0, D3, D5, true), 
// 						loopTime, 0.001f, 0.0f, 0.0f);


// Right striker Actuator Init
RotaryActuator StrikerR(new QEI(PD_4, PD_3, NC, STRIKER_ENCODER_RESOLUTION),
						new SingleMC33926MotorController(D8, D14, A2, PB_10, PE_15, false), 
						loopTime, 0.0015f, 0.0f, 0.0002f);

/*

// Left striker Actuator Init
RotaryActuator StrikerL(new QEI(PD_6, PD_5, NC, STRIKER_ENCODER_RESOLUTION),
						new SingleMC33926MotorController(D7, D13, A1, PE_14, PB_11, false), 
						loopTime, 0.001f, 0.0f, 0.0f);

*/

int main(){
	
	// thread.start(callback(&eventQueue, &EventQueue::dispatch_forever));
	// eventQueue.call_every(1000, printf, "Hello from the thread\n");

	while(!stopButton){
		// printf("Dampener Pos: %d\tStrikerR Pos: %d\tStrikerL Pos: %d\n", Dampener.readPos(), StrikerR.readPos(), StrikerL.readPos());
		// printf("StrikerR Pos: %d\n", StrikerR.readPos());
	}


	// pc.puts("calibrating dampener now...\n");
	// Dampener.calibrate(200);
	// pc.puts("Done calibrating dampener\n");

	pc.puts("calibrating right striker now...\n"); // TODO: Figure out why right striker encoder is not being read
	StrikerR.calibrate(-200);
	pc.puts("Done calibrating right striker now\n");

	// pc.puts("calibrating left striker now...\n");
	// StrikerL.calibrate(-200);
	// pc.puts("Done calibrating left striker\n");

	wait(5);
	while(1){
		StrikerR.coastStrike(0.6, 100, 3.0f);
		wait(5);
		// StrikerR.setPosSetpoint(0);
		// wait(5);
		// StrikerR.setPosSetpoint(200);
		// wait(5);
		// pc.puts("done striking\n");
		// printf("Encoder Pose at home: %d\n", StrikerL.readEncoder());
	}



	return 0;
}

void dampenBlocking(RotaryActuator Actuator){
	Actuator.setPosSetpoint(20);
	wait(5);
	Actuator.setPosSetpoint(-200);
	wait(5);
}