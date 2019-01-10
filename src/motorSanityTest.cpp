#include <mbed.h>
#include "RotaryActuator.h"

// Init variables
int loopTime = 3; // milliseconds

// Communication
Serial pc(USBTX, USBRX);

// User button for test program flow
DigitalIn stopButton(USER_BUTTON);

// SingleMC33926MotorController * RightStriker = new SingleMC33926MotorController(D8, D14, A2, PB_10, PE_15, false);
// QEIx4 * RightEncoder = new QEIx4(PD_4, PD_3, NC, (QEIx4::EMODE)(QEIx4::IRQ | QEIx4::SPEED));


// Right striker Actuator Init
// RotaryActuator StrikerR(new QEIx4(PD_4, PD_3, NC, (QEIx4::EMODE)(QEIx4::IRQ | QEIx4::SPEED)),
// 						new SingleMC33926MotorController(D8, D14, A2, PB_10, PE_15, false), 
// 						loopTime, 0.0004f, 0.0f, 0.00004f);

// Left striker Actuator Init
RotaryActuator StrikerL(new QEIx4(PD_6, PD_5, NC, (QEIx4::EMODE)(QEIx4::IRQ | QEIx4::SPEED)),
						new SingleMC33926MotorController(D7, D13, A1, PE_14, PB_11, false), 
						loopTime, 0.0006f, 0.0f, 0.00004f);


// Dampener Actuator Init
RotaryActuator Dampener(new QEIx4(PD_1, PD_0, NC, (QEIx4::EMODE)(QEIx4::IRQ | QEIx4::SPEED)),
						new SingleMC33926MotorController(D11, D15, A0, D3, D5, true), 
						loopTime, 0.0006f, 0.0f, 0.0f);


Thread t;
EventQueue queue;


int main(){
	
	Dampener.setCurrentOffsetThreshold(0.015f);

	while(!stopButton);
	/*
	RightStriker->enable();
	RightStriker->setSpeedCoast(0.9f);
	wait(1);
	RightStriker->setSpeedCoast(0.0f);
	wait(3);
	RightStriker->setSpeedBrake(-0.4f);
	wait(1);
	RightStriker->setSpeedBrake(0.0f);
	
	*/

	// // Start the event queue's dispatch thread
	t.start(callback(&queue, &EventQueue::dispatch_forever));

	// // _tick.attach(this, &RotaryActuator::controlLoop, controlInterval);
	// queue.call_every(loopTime, &StrikerR, &RotaryActuator::controlLoop);
	queue.call_every(loopTime, &StrikerL, &RotaryActuator::controlLoop);
	queue.call_every(loopTime, &Dampener, &RotaryActuator::controlLoop);


	// pc.puts("calibrating right striker now...\n"); // TODO: Figure out why right striker encoder is not being read
	// StrikerR.calibrate(650);
	// pc.puts("Done calibrating right striker now\n");
	
	pc.puts("calibrating left striker now...\n");
	StrikerL.calibrate(650);
	pc.puts("Done calibrating left striker\n");

	pc.puts("calibrating dampener now...\n");
	Dampener.calibrate(400);
	pc.puts("Done calibrating dampener\n");
	


	wait(2);


	while(!stopButton){
		// StrikerR.coastStrike(0.85, 350, 500);
		// wait(0.5);
		StrikerL.coastStrike(0.7, 400, 500);
		wait(0.5);
		Dampener.goToString();
		wait(0.5);
		Dampener.goHome();
		
		// StrikerR.goHome();
		// StrikerR.setPosSetpoint(0);
		// wait(5);
		// StrikerR.goHome();
		wait(5);

	}
	
	return 0;
}