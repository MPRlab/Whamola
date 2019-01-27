#include "mbed.h"
#include "EthernetInterface.h"
#include "osc_client.h"
#include "ODriveMbed.h"
#include "RotaryActuator.h"
#include <string>
#include "StringTuner.h"


#define ODRIVE2TX D1
#define ODRIVE2RX D0

DigitalOut ADCTestPin(PE_4);

// User button for test program flow
DigitalIn userButton(USER_BUTTON);

// pc UART comms constructor
Serial pc(USBTX, USBRX);

// ODrive constructor
Serial * odrive_serial = new Serial(ODRIVE2TX, ODRIVE2RX);

// ACF Constructor
Yin ACF(A3);

// String Tuner constructor
StringTuner TuningHead(odrive_serial, &ACF, 21, 38);

// Init variables
int loopTime = 5; // milliseconds
Thread thread(osPriorityHigh);
// Thread AudioThread(osPriorityNormal);

EventQueue queue;

Timer t;
/**
 * Main function
 */
int main() {
	pc.baud(115200);

	while(!userButton){}

	ADCTestPin = 1;
	// Start the event queue's dispatch thread
    pc.puts("starting thread now\n");
	thread.start(callback(&queue, &EventQueue::dispatch_forever));
	// queue.call_every(PERIOD_ACF*1000, &ACF, &Yin::readSample);

	// AudioThread.start(&ACF, &Yin::readSample);
	pc.puts("thread just started\n");
	// calibrateStrikers();
	// wait_ms(500);
	// printf("added readSample to queue\n");
	// TuningHead.autoStringCalibration(&StrikerL);
	
	t.start();
	float measFreq, timeT;
	while(1){
        ACF.readSample();
		measFreq = ACF.FreqCalc();
		timeT = t.read();
		printf("Time: %f\tRandom Reading: %f\tFrequency: %f\n", timeT, ACF.rawData[500], measFreq);
	}

	printf("Starting Instrunemt\r\n");

}