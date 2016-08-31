
#ifndef MedianFilter_h
#define MedianFilter_h

#include "Arduino.h"

class MedianFilter
{
public:
	MedianFilter(byte size, int seed);
	int in(int value);
	int out(); 
	void printData();		// used for debugging
	
private:
	int filterShift;
	byte medFilterWin;		// number of samples in sliding median filter window - usually odd #
	byte medDataPointer;	// mid point of window
	int * sortedData;		// array pointer for data sorted by size
	byte * historyMap;		// array pointer for locations of history data in sorted list
	byte * locationMap;		// array pointer for data locations in history map
	int ODP;				// oldest data point in accelRawHistory
	int tempData;			// temp data storage while swapping data locations
	byte tempMap;			// temp map storage while swapping data locations
	boolean dataMoved;
};

#endif
