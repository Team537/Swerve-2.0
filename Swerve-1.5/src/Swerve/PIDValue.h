#ifndef SWERVE_PIDVALUE_H_
#define SWERVE_PIDVALUE_H_

#include <WPILib.h>
#include <Schematic.h>

class PIDValue {
private:
	float p, i, d, inputMin, inputMax, outputMin, outputMax;

public:
	PIDValue(float p, float i, float d, float mininput, float maxinput, float minoutput, float maxoutput) {
		this->p = p;
		this->i = i;
		this->d = d;
		this->inputMin = mininput;
		this->inputMax = maxinput;
		this->outputMin = minoutput;
		this->outputMax = maxoutput;
	}

	float getD() {
		return d;
	}

	float getI() {
		return i;
	}

	float getInputMax() {
		return inputMax;
	}

	float getInputMin() {
		return inputMin;
	}

	float getOutputMax() {
		return outputMax;
	}

	float getOutputMin() {
		return outputMin;
	}

	float getP() {
		return p;
	}
};

#endif
