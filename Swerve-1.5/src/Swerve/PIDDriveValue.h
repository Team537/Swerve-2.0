#ifndef SWERVE_PIDDRIVEVALUE_H_
#define SWERVE_PIDDRIVEVALUE_H_

#include <WPILib.h>
#include <Schematic.h>

class PIDDriveValue {
private:
	float p, i, d, f, rate;

public:
	PIDDriveValue(float p, float i, float d, float f, float rate) {
		this->p = p;
		this->i = i;
		this->d = d;
		this->f = f;
		this->rate = rate;
	}

	float getD() {
		return d;
	}

	float getF() {
		return f;
	}

	float getI() {
		return i;
	}

	float getP() {
		return p;
	}

	float getRate() {
		return rate;
	}
};

#endif
