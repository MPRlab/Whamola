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

/*

// Right striker Actuator Init
RotaryActuator StrikerR(new QEI(PD_4, PD_3, NC, STRIKER_ENCODER_RESOLUTION),
						new SingleMC33926MotorController(D8, D14, A2, PB_10, PE_15, false), 
						loopTime, 0.0015f, 0.0f, 0.0002f);

*/

QEI RightEnc(PD_4, PD_3, NC, STRIKER_ENCODER_RESOLUTION);
SingleMC33926MotorController RightMotor(D8, D14, A2, PB_10, PE_15, false);

/*

// Left striker Actuator Init
RotaryActuator StrikerL(new QEI(PD_6, PD_5, NC, STRIKER_ENCODER_RESOLUTION),
						new SingleMC33926MotorController(D7, D13, A1, PE_14, PB_11, false), 
						loopTime, 0.001f, 0.0f, 0.0f);

*/

Timer t, controlLoop;
arm_pid_instance_f32 * _ArmPosPid;

int main(){
	
	// thread.start(callback(&eventQueue, &EventQueue::dispatch_forever));
	// eventQueue.call_every(1000, printf, "Hello from the thread\n");
	
	// Set PID Parameters following this forum post (https://os.mbed.com/questions/1904/mbed-DSP-Library-PID-Controller/)
	_ArmPosPid->Kp = 0.0015;
	_ArmPosPid->Ki = 0.0;
	_ArmPosPid->Kd = 0.002;
	arm_pid_init_f32(_ArmPosPid, true); // Passes in DampPID_cmsis struct and set reset flag to true



	t.start();
	controlLoop.start();
	int setpoint = 3200;

	pc.printf("starting\n");

	while(!stopButton){
		while(t.read() < 5 && !stopButton){
			controlLoop.reset();
			int pos = RightEnc.getPulses();
			float error = (float) setpoint - pos;
			float out = - arm_pid_f32(_ArmPosPid, error);
			RightMotor.setSpeedCoast(out);
			pc.printf("PID output: %f\tPosition: %d\tSetpoint: %d\tError: %f\n", out, pos, setpoint, error);
			while(controlLoop.read_ms() < loopTime);
		}
		
		t.reset();
		setpoint = 0;

		while(t.read() < 5 && !stopButton){
			controlLoop.reset();
			int pos = RightEnc.getPulses();
			float error = (float) setpoint - pos;
			float out = - arm_pid_f32(_ArmPosPid, error);
			RightMotor.setSpeedCoast(out);
			pc.printf("PID output: %f\tPosition: %d\tSetpoint: %d\tError: %f\n", out, pos, setpoint, error);
			while(controlLoop.read_ms() < loopTime);
		}
		t.reset();
		setpoint = 3200;

	}
	RightMotor.setSpeedCoast(0.0f);


	return 0;
}
