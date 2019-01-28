#include "StringTuner.h"

	StringTuner::StringTuner(Serial * odrive_serial, int lowestNote, int highestNote){

	// Initilialize ODrive
	printf("setting baud rate\r\n");
	odrive_serial->baud(115200); 
	printf("Creating ODrive object\r\n");
	_odrive = new ODriveMbed(*odrive_serial);
	_axis = 0;


	populateIdealNoteTable();

	if(lowestNote < 21){
		_lowestNote = lowestNote;
		printf("The lower bound cannot be lower than an A0, setting lowestNote to A0\n");
	}
	else
		_lowestNote = lowestNote;

	if(_idealNotesMap.find(highestNote) == _idealNotesMap.end()){
		_highestNote = _idealNotesMap.rbegin()->first;
		printf("There is currently no entry for this, defaulting to the highest note entered into the hashmap\n");
	}	
	else
		_highestNote = highestNote;


}

vector<float> StringTuner::updateRegression(){
	float measuredFreq = FreqCalc();
	float measuredPose = _odrive->getPositionEstimate(_axis);
	return updateModel(measuredPose, pow(measuredFreq, 2));
}


void StringTuner::playMidiNote(int noteNumber){
	_currentNote = noteNumber;
	float freqDesired = _idealNotesMap[noteNumber];
	float desiredPosition = inverseCalculation(pow(freqDesired, 2));
	_currentNoBendPose = desiredPosition;
	updateODrivePosition(desiredPosition);

	// Find the neighboring semitone Poses for pitch bend purposes
	_lowerSemitonePose = inverseCalculation(pow(_idealNotesMap[noteNumber - 1], 2));
	_uppperSemitonePose = inverseCalculation(pow(_idealNotesMap[noteNumber + 1], 2));
}

void StringTuner::pitchBend(int bendValue){
	float bendDist, distBetweenNotes;
	// value from 0-127 where 63 is no bend, 0 is the semi-tone below, and 127 the semi-tone above
	if(bendValue == 63)
		updateODrivePosition(_currentNoBendPose);
	else if(bendValue < 63){
		distBetweenNotes = abs(_currentNoBendPose - _lowerSemitonePose);
		bendDist = - (distBetweenNotes / 64) * bendValue;
	}
	else{ // bendValue > 63
		distBetweenNotes = abs(_currentNoBendPose - _uppperSemitonePose);
		bendDist = (distBetweenNotes / 64) * bendValue;
	}
	updateODrivePosition(_currentNoBendPose + bendDist);
}

void StringTuner::updateODrivePosition(float position){
	_odrive->setPosition(_axis, position);

}

void StringTuner::updateODrivePositionUser(float position){
	_currentNoBendPose = position;
	_odrive->setPosition(_axis, _zeroCurrentPose + position);
}


void StringTuner::calibrateODrive(){
	int axis = 0;

	printf("Checking ODrive now...\n");
	while(_odrive->readState(_axis) != ODriveMbed::AXIS_STATE_CLOSED_LOOP_CONTROL){
		wait(0.1);
	}

	_odrive->setControlMode(_axis, ODriveMbed::CTRL_MODE_CURRENT_CONTROL, false);
	wait(1); // Hopefully this actually changes to Current control
	_odrive->setCurrent(_axis, 2);
	wait(1);
	_odrive->setCurrent(_axis, 0);
	_zeroCurrentPose = _odrive->getPositionEstimate(axis);

	// Set position to be zero to be safe
	_odrive->setPosition(_axis, 0);
	wait(2);
	// Now switch back to position control
	if(!_odrive->setControlMode(_axis, ODriveMbed::CTRL_MODE_POSITION_CONTROL, true))
		printf("something went wrong and the control mode was not successfully set, current mode is a %d\n", _odrive->readControlMode(_axis));
	else
		printf("control mode set to: %d\n", _odrive->readControlMode(_axis));
	wait(1);

	_odrive->setPosition(_axis, (int) _zeroCurrentPose - 500);
	printf("setting odrive to %d\n", (int) _zeroCurrentPose + 5000);
	_currentNoBendPose = _zeroCurrentPose - 500.0f;
	// TODO: Add calbration sequence that will find different note values
}


void StringTuner::autoStringCalibration(RotaryActuator * Striker){
	int pose = _zeroCurrentPose - 500.0f;
	float measuredFreq = 0.0f;
	float measuredPose;
	printf("starting for loop now\n");
	while (measuredFreq < _idealNotesMap[_highestNote]){
		updateODrivePosition(pose);
		wait_ms(200);
		Striker->coastStrikeMIDI(60);
		printf("Just struck string\n");
		wait_ms(500);
		printf("calculating frequency\n");
		printf("ACF frequency = %f\n",FreqCalc());
		measuredFreq += 10.0f; // substitute this for now
		printf("Reading Lever Position\n");
	 	_currentNoBendPose = _odrive->getPositionEstimate(_axis);
	 	updateModel(_currentNoBendPose, pow(measuredFreq, 2));
	 	wait_ms(500);
	 	pose += 500.0f;
	}
}


void StringTuner::populateIdealNoteTable(){

	// 0th Octave
	_idealNotesMap.insert(pair<int, float>(21, 27.50f)); // A0
	_idealNotesMap.insert(pair<int, float>(22, 29.14f)); // A#0/Bb0
	_idealNotesMap.insert(pair<int, float>(23, 30.87f)); // B0

	// 1st Octave
	_idealNotesMap.insert(pair<int, float>(24, 32.70f)); // C1
	_idealNotesMap.insert(pair<int, float>(25, 34.65f)); // C#1/Db1
	_idealNotesMap.insert(pair<int, float>(26, 36.71f)); // D1
	_idealNotesMap.insert(pair<int, float>(27, 38.89f)); // D#1/Eb1
	_idealNotesMap.insert(pair<int, float>(28, 41.20f)); // E1
	_idealNotesMap.insert(pair<int, float>(29, 43.65f)); // F1
	_idealNotesMap.insert(pair<int, float>(30, 46.25f)); // F#1/Gb1
	_idealNotesMap.insert(pair<int, float>(31, 49.00f)); // G1
	_idealNotesMap.insert(pair<int, float>(32, 51.91f)); // G#1/Ab1
	_idealNotesMap.insert(pair<int, float>(33, 55.00f)); // A1
	_idealNotesMap.insert(pair<int, float>(34, 58.27f)); // A#1/Bb1
	_idealNotesMap.insert(pair<int, float>(35, 61.74f)); // B1
	
	// 2nd Octave
	_idealNotesMap.insert(pair<int, float>(36, 65.41f)); // C2
	_idealNotesMap.insert(pair<int, float>(37, 69.30f)); // C#2/Db2
	_idealNotesMap.insert(pair<int, float>(38, 73.42f)); // D2
	_idealNotesMap.insert(pair<int, float>(39, 77.78f)); // D#2/Eb2
	_idealNotesMap.insert(pair<int, float>(40, 82.41f)); // E2
	_idealNotesMap.insert(pair<int, float>(41, 87.31f)); // F2
	_idealNotesMap.insert(pair<int, float>(42, 92.50f)); // F#2/Gb2
	_idealNotesMap.insert(pair<int, float>(43, 98.00f)); // G2
	_idealNotesMap.insert(pair<int, float>(44, 103.83f)); // G#2/Ab2
	_idealNotesMap.insert(pair<int, float>(45, 110.00f)); // A2
	_idealNotesMap.insert(pair<int, float>(46, 116.54f)); // A#2/Bb2
	_idealNotesMap.insert(pair<int, float>(47, 123.47f)); // B2

	// 3rd Octave
	_idealNotesMap.insert(pair<int, float>(48, 130.81f)); // C3
	_idealNotesMap.insert(pair<int, float>(49, 138.59f)); // C#3/Db3
	_idealNotesMap.insert(pair<int, float>(50, 146.83f)); // D3
	_idealNotesMap.insert(pair<int, float>(51, 155.56f)); // D#2/Eb2
	_idealNotesMap.insert(pair<int, float>(52, 164.81f)); // E3
	_idealNotesMap.insert(pair<int, float>(53, 174.61f)); // F3
	_idealNotesMap.insert(pair<int, float>(54, 185.00f)); // F#3/Gb3
	_idealNotesMap.insert(pair<int, float>(55, 196.00f)); // G3
	_idealNotesMap.insert(pair<int, float>(56, 207.65f)); // G#3/Ab3
	_idealNotesMap.insert(pair<int, float>(57, 220.00f)); // A3
	_idealNotesMap.insert(pair<int, float>(58, 233.08f)); // A#3/Bb3
	_idealNotesMap.insert(pair<int, float>(59, 246.94f)); // B3
	

	// ENTER HIGHER NOTES HERE IF NEEDED 		
}