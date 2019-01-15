#include "mbed.h"


typedef struct NoteMapValueStruct{
	float odriveEncEstimate;
	int midiNote;
	float Tension;
} NoteMapValue;

std::map<int midiNote, float idealFreq> idealNotesMap;

std::map<int measuredFreq, NoteMapValue value> robotFreqMap;

