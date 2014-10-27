/*
 * timing.cpp
 *
 *  Created on: Jul 10, 2014
 *      Author: lbarnett
 */

#include "../include/timing.h"

timing::timing(std::string name, int samples) {
	this->name = name;
	this->samples = samples;
	total = 0;
	average = 0;
	sampleCount = 0;

}

timing::~timing() {
	// TODO Auto-generated destructor stub
}
 void timing::startSample() {
	 if (samples == 0)
		 return;

	 gettimeofday(&startTime, NULL);
 }

 void timing::stopSample() {
	 if (samples == 0)
		 return;

	 timeval endTime;
	 gettimeofday(&endTime, NULL);
	 calculateAverage(endTime);
 }

 void timing::calculateAverage(timeval endTime) {
	 long timeTaken = (endTime.tv_sec - startTime.tv_sec) * 1000000 + (endTime.tv_usec - startTime.tv_usec);

	 if (sampleCount == samples) {
		 long oldSample = sampleQueue.front();
		 sampleQueue.pop();

		 total -= oldSample;
	 } else {
		 sampleCount++;
	 }
	 total += timeTaken;
	 average = total / sampleCount;
	 sampleQueue.push(timeTaken);
 }

 long timing::averageTime() {
	 return average;
 }
