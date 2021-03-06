#include "Swerve.hpp"

void Swerve::Drive() {
	double fwd = fabs(joystick->GetX()) >= JOYSTICK_DEADBAND ? joystick->GetX() : 0;
	double str = fabs(joystick->GetY()) >= JOYSTICK_DEADBAND ? joystick->GetY() : 0;
	double rcw = fabs(joystick->GetZ()) >= JOYSTICK_DEADBAND ? joystick->GetZ() : 0;
	double gro = gyro->GetAngle();

	double fwd2 = (fwd * cos(gro)) + str * sin(gro);
	double str2 = (-fwd * sin(gro)) + str * cos(gro);

	double r = sqrt(WHEEL_BASE * WHEEL_BASE + TRACK_WIDTH * TRACK_WIDTH) / 2;
	double a = str2 - rcw * ((WHEEL_BASE / r) * 0.5);
	double b = str2 + rcw * ((WHEEL_BASE / r) * 0.5);
	double c = fwd2 - rcw * ((TRACK_WIDTH / r) * 0.5);
	double d = fwd2 + rcw * ((TRACK_WIDTH / r) * 0.5);

	SmartDashboard::PutNumber("Inputs Forward", fwd);
	SmartDashboard::PutNumber("Inputs Strafe", str);
	SmartDashboard::PutNumber("Inputs Rotation", rcw);
	SmartDashboard::PutNumber("Inputs Gyro", gro);

	SmartDashboard::PutNumber("Maths R", r);
	SmartDashboard::PutNumber("Maths A", a);
	SmartDashboard::PutNumber("Maths B", b);
	SmartDashboard::PutNumber("Maths C", c);
	SmartDashboard::PutNumber("Maths D", d);

	double frs = sqrt(b * b + c * c);
	double fls = sqrt(a * a + c * c);
	double bls = sqrt(a * a + d * d);
	double brs = sqrt(b * b + d * d);

	double fra = atan2(c, b) * 180 / PI;
	double fla = atan2(c, a) * 180 / PI;
	double bla = atan2(d, a) * 180 / PI;
	double bra = atan2(d, b) * 180 / PI;

	double max = frs;

	if (fls > max) {
		max = fls;
	}

	if (bls > max) {
		max = bls;
	}

	if (brs > max) {
		max = brs;
	}

	if (max > 1) {
		frs /= max;
		fls /= max;
		bls /= max;
		brs /= max;
	}

	frontRight->Drive(frs, fra);
	frontLeft->Drive(fls, fla);
	backLeft->Drive(bls, bla);
	backRight->Drive(brs, bra);
}

void Swerve::Tune(SwerveModule *module) {
	CANTalon* talon = module->getAngleTalon();

	if (tuningButton->WasDown()) {
		if (talon->IsControlEnabled()) {
			talon->Disable();
		} else {
			talon->EnableControl();
		}
	}

	SmartDashboard::PutNumber("Tuning Error", talon->GetSetpoint() - talon->GetAnalogInRaw());
	SmartDashboard::PutNumber("Tuning Setpoint", talon->GetSetpoint());
	SmartDashboard::PutBoolean("Tuning Enabled", talon->IsControlEnabled());
}

void Swerve::Dashboard() {
	frontRight->Dashboard();
	frontLeft->Dashboard();
	backLeft->Dashboard();
	backRight->Dashboard();
}
