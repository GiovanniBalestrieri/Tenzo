
#ifndef MedianFilter_h
#define MedianFilter_h

#include "Arduino.h"

class MedianFilter
{
public:
	MedianFilter(byte size, int seed);
	int in( volatile int value);
	int out(); 
	//void printData();		// used for debugging
	
private:
	volatile int filterShift;
	volatile byte medFilterWin;		// number of samples in sliding median filter window - usually odd #
	volatile byte medDataPointer;	// mid point of window
	volatile int * sortedData;		// array pointer for data sorted by size
	volatile byte * historyMap;		// array pointer for locations of history data in sorted list
	volatile byte * locationMap;		// array pointer for data locations in history map
	volatile int ODP;				// oldest data point in accelRawHistory
	volatile int tempData;			// temp data storage while swapping data locations
	volatile byte tempMap;			// temp map storage while swapping data locations
	volatile boolean dataMoved;
};

#endif
