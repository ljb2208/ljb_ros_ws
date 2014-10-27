/*
 * timing.h
 *
 *  Created on: Jul 10, 2014
 *      Author: lbarnett
 */

#ifndef TIMING_H_
#define TIMING_H_

#include <string>
#include <queue>
#include <sys/time.h>

class timing {
public:
	timing(std::string name, int samples);
	virtual ~timing();
	void startSample();
	void stopSample();
	long averageTime();


private:
	std::string name;
	int samples;
	int sampleCount;
	timeval startTime;
	std::queue<long> sampleQueue;
	long total;
	long average;

	void calculateAverage(timeval endTime);
};

#endif /* TIMING_H_ */
