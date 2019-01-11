#include "ODriveMbed.h"
#include "RotaryActuator.h"
#include "mbed.h"

#define ODRIVE2TX D1
#define ODRIVE2RX D0

// pc UART comms constructor
Serial pc(USBTX, USBRX);

// User button for test program flow
DigitalIn stopButton(USER_BUTTON);

// ODrive constructor
Serial odrive_serial(ODRIVE2TX, ODRIVE2RX);
ODriveMbed odrive(odrive_serial);


// Init variables
int loopTime = 3; // milliseconds
Thread thread(osPriorityHigh);
EventQueue queue;

// Right striker Actuator Init
RotaryActuator StrikerR(new QEIx4(PD_4, PD_3, NC, (QEIx4::EMODE)(QEIx4::IRQ | QEIx4::SPEED)),
						new SingleMC33926MotorController(D8, D14, A2, PB_10, PE_15, false), 
						loopTime, 0.0005f, 0.0f, 0.0001f);
// SingleMC33926MotorController * rightStriker =  new SingleMC33926MotorController(D8, D14, A2, PB_10, PE_15, false);


// Left striker Actuator Init
RotaryActuator StrikerL(new QEIx4(PD_6, PD_5, NC, (QEIx4::EMODE)(QEIx4::IRQ | QEIx4::SPEED)),
						new SingleMC33926MotorController(D7, D13, A1, PE_14, PB_11, false), 
						loopTime, 0.0005f, 0.0f, 0.0001f);


// Dampener Actuator Init
RotaryActuator Dampener(new QEIx4(PD_1, PD_0, NC, (QEIx4::EMODE)(QEIx4::IRQ | QEIx4::SPEED)),
						new SingleMC33926MotorController(D11, D15, A0, D3, D5, true), 
						loopTime, 0.001f, 0.0f, 0.0004f);


int main(){
	pc.baud(115200);
	odrive_serial.baud(115200);
	Dampener.setCurrentOffsetThreshold(0.008f);


    while(!stopButton);

	int axis = 0;

	pc.puts("Checking ODrive now...\n");
	if(!odrive.setControlMode(axis, ODriveMbed::CTRL_MODE_POSITION_CONTROL, true))
		pc.printf("something went wrong and the control mode was not successfully set, current mode is a %d\n", odrive.readControlMode(axis));
	else
		pc.printf("control mode set to: %d\n", odrive.readControlMode(axis));
		

	// // Start the event queue's dispatch thread
	thread.start(callback(&queue, &EventQueue::dispatch_forever));

	// attach the Striker and Dampener controlLoop functions to the EventQueue to repeat on the loop time
	queue.call_every(loopTime, &StrikerR, &RotaryActuator::controlLoop);
	queue.call_every(loopTime, &StrikerL, &RotaryActuator::controlLoop);
	queue.call_every(loopTime, &Dampener, &RotaryActuator::controlLoop);

	pc.puts("calibrating right striker now...\n"); // TODO: Figure out why right striker encoder is not being read
	StrikerR.calibrate(700);
	pc.puts("Done calibrating right striker now\n");
	
	pc.puts("calibrating left striker now...\n");
	StrikerL.calibrate(700);
	pc.puts("Done calibrating left striker\n");

	pc.puts("calibrating dampener now...\n");
	Dampener.calibrate(400);
	pc.puts("Done calibrating dampener\n");


	wait(5);


	while(1){
		// printf("State of the striker Thread: %d\n",thread.get_state());
		Dampener.goToString();
		wait(0.5);
		Dampener.goHome();
		wait(0.5);
		odrive.setPosition(axis, 14000);
		StrikerL.coastStrike(0.75, 350, 500);
		// wait(0.25);
		// StrikerR.coastStrike(0.8, 400, 500);
		wait(2.5);
		odrive.setPosition(axis, 20000);
		StrikerL.coastStrike(0.85, 250, 500);
		// wait(0.25);
		// StrikerR.coastStrike(0.7, 400, 500);
		wait(0.75);
		Dampener.goToString();
		wait(1.0);
		Dampener.goHome();
		wait(0.5);
	}
}
