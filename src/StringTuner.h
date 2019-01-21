#ifndef StringTuner_h
#define StringTuner_h

#include "mbed.h"
#include <map>
#include "LinearRegression.h"
#include "ODriveMbed.h"

// TODO: include and add the frequency detecting 
using namespace std;

class StringTuner : public LinearRegression{
public:

	StringTuner(Serial odrive_serial, int lowestNote, int highestNote); // specify the ideal notes hashmap in here

	ODriveMbed * _odrive;


	typedef struct NoteMapValueStruct{
		float odriveEncEstimate;
		float measuredFrequency;
		float tension;
		float motorCurrent;
	} NoteMapValue;

	std::map<int, float> _idealNotesMap;

	std::map<float, NoteMapValue> _robotFreqMap;

	// Public Methods
	void playMidiNote(int noteNumber);
	void pitchBend(int bendValue); // value from 0-127 where 63 is no bend, 0 is the semi-tone below, and 127 the semi-tone above

private:

	// Private members
	int _lowestNote, _highestNote, _currentNote;
	// Odrive stuff
	float _currentNoBendPose, _lowerSemitonePose, _uppperSemitonePose;
	int _axis;

	// Private Methods
	void populateIdealNoteTable();
	float midi2freq(int noteNumber);
	void updateODrivePosition(float position);

};
#endif