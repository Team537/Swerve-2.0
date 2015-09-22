#ifndef SWERVE_PIDDISTANCE_H_
#define SWERVE_PIDDISTANCE_H_

#include <WPILib.h>
#include <Schematic.h>

class PIDDistance {
private:
	float p, i, d;

public:
	PIDDistance(float p, float i, float d) {
		this->p = p;
		this->i = i;
		this->d = d;
	}

	float getD() {
		return d;
	}

	float getI() {
		return i;
	}

	float getP() {
		return p;
	}
};

#endif
