#ifndef SWERVE_DOUBLEPOT_H_
#define SWERVE_DOUBLEPOT_H_

#include <cmath>
#include <WPIlib.h>
#include <Schematic.h>

#define OVERSAMPLE 2

class DoublePot: public AnalogPotentiometer {
private:
	bool alternater, lastDeltaSign;
	double lastValue;
	float min, max;
	int counter;
	double accumulator, offset;
	std::string name;
	int oversampleAccumulator;
	double data[OVERSAMPLE];

public:
	DoublePot(int channel, double fullRange, float min, float max, float offset, std::string name) : AnalogPotentiometer(channel, fullRange, offset) {
		this->alternater = false;
		this->lastDeltaSign = false;
		this->lastValue = 0;
		this->min = min;
		this->max = max;
		this->counter = 0;
		this->accumulator = 0;
		this->offset = offset;
		this->name = name;
		this->oversampleAccumulator = 0;

		for (int i = 0; i < OVERSAMPLE; i++) {
			this->data[i] = 0;
		}
	}

	double PIDGet();
	double GetAverage();
};

#endif
