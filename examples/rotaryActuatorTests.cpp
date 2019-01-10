#include <mbed.h>
#include "RotaryActuator.h"
// #include "SingleMC33926MotorController.h"
// #include <QEI.h>

#define STRIKER_ENCODER_RESOLUTION 3200

// Init variables
int setpoint = -100;
int loopTime = 10; // milliseconds

// Code taken from the mbed eventqueue API reference page:
// Create a queue that can hold a maximum of 32 events
EventQueue queue;
// Create a thread that'll run the event queue's dispatch function
Thread t;


// Communication
Serial pc(USBTX, USBRX);

// User button for shutoff
DigitalIn stopButton(USER_BUTTON);


// Dampener Actuator Init
// RotaryActuator Dampener(new QEIx4(PD_1, PD_0, NC, (QEIx4::EMODE)(QEIx4::IRQ | QEIx4::SPEED)),
// 						new SingleMC33926MotorController(D11, D15, A0, D3, D5, true), 
// 						loopTime, 0.0005f, 0.0f, 0.0f);


// Right striker Actuator Init
RotaryActuator StrikerR(new QEIx4(PD_4, PD_3, NC, (QEIx4::EMODE)(QEIx4::IRQ | QEIx4::SPEED)),
						new SingleMC33926MotorController(D8, D14, A2, PB_10, PE_15, false), 
						loopTime, 0.0006f, 0.0f, 0.0001f);



// Left striker Actuator Init
RotaryActuator StrikerL(new QEIx4(PD_6, PD_5, NC, (QEIx4::EMODE)(QEIx4::IRQ | QEIx4::SPEED)),
						new SingleMC33926MotorController(D7, D13, A1, PE_14, PB_11, false), 
						loopTime, 0.0006f, 0.0f, 0.0001f);



int main(){




	pc.puts("started\n");
	while(!stopButton){
		// printf("Dampener Pos: %d\tStrikerR Pos: %d\tStrikerL Pos: %d\n", Dampener.readPos(), StrikerR.readPos(), StrikerL.readPos());
		pc.printf("StrikerR Pos: %f\n", StrikerR.readPos());
		
	}

	pc.puts("calibrating left striker now...\n");
	StrikerL.calibrate(500);
	pc.puts("Done calibrating left striker\n");

	// pc.puts("calibrating dampener now...\n");
	// Dampener.calibrate(500);
	// pc.puts("Done calibrating dampener\n");

	pc.puts("calibrating right striker now...\n"); // TODO: Figure out why right striker encoder is not being read
	StrikerR.calibrate(500);
	pc.puts("Done calibrating right striker now\n");



	// Start the event queue's dispatch thread
	t.start(callback(&queue, &EventQueue::dispatch_forever));

	// _tick.attach(this, &RotaryActuator::controlLoop, controlInterval);
	queue.call_every(loopTime, &StrikerL, &RotaryActuator::controlLoop);
	// queue.call_every(loopTime, &StrikerR, &RotaryActuator::controlLoop);
	// queue.call_every(loopTime, &Dampener, &RotaryActuator::controlLoop);



	wait(5);
	while(1){
		// StrikerR.coastStrike(0.5, 300, 500);
		StrikerL.coastStrike(0.5, 300, 1000);
		wait(10);

		// Dampener.setPosSetpoint(20);
		// wait(5);	
		// Dampener.goHome();
	
		// pc.puts("done striking\n");
		// printf("Encoder Pose at home: %d\n", StrikerL.readEncoder());
	}



	return 0;
}
