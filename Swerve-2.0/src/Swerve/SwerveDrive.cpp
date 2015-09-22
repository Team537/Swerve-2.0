#include <Swerve/SwerveDrive.h>

void SwerveDrive::Initalize() {

}

void SwerveDrive::Update() {
	double FWD = joystick->GetY();
	double STR = joystick->GetX();
	double RCW = joystick->GetZ();
	double GRO = FIELD_ORIENTED ? gyro->GetAngle() : 0;

	double FWD2 = (FWD * cos(GRO)) + STR * sin(GRO);
	double STR2 = (-FWD * sin(GRO)) + STR * cos(GRO);

	double w = ROBOT_WIDTH;
	double h = ROBOT_LENGTH;
	double r = sqrt((w * w) + (h * h));

	double a = STR2 - RCW * (w / r);
	double b = STR2 + RCW * (w / r);
	double c = FWD2 - RCW * (h / r);
	double d = FWD2 + RCW * (h / r);

	double frs = sqrt(b * b + c * c);
	double fls = sqrt(b * b + d * d);
	double rls = sqrt(a * a + d * d);
	double rrs = sqrt(a * a + c * c);
	double fra = atan2(b, c) * 180 / PI;
	double fla = atan2(b, d) * 180 / PI;
	double rra = atan2(a, d) * 180 / PI;
	double rla = atan2(a, c) * 180 / PI;

	double max = frs;

	if (fls > max) {
		max = fls;
	}

	if (rls > max) {
		max = rls;
	}

	if (rrs > max) {
		max = rrs;
	}

	if (max > 1) {
		frs /= max;
		fls /= max;
		rrs /= max;
		rls /= max;
	}

	fra = (fra + 180) * (1023 / 360);
	fla = (fla + 180) * (1023 / 360);
	rla = (rla + 180) * (1023 / 360);
	rra = (rra + 180) * (1023 / 360);

	FRSteer->Set(fra);
	FLSteer->Set(fla);
	RLSteer->Set(rla);
	RRSteer->Set(rra);

	FRDrive->Set(frs);
	FLDrive->Set(fls);
	RRDrive->Set(rrs);
	RLDrive->Set(rls);

	SmartDashboard::PutNumber("FWD: ", FWD);
	SmartDashboard::PutNumber("STR: ", STR);
	SmartDashboard::PutNumber("RCW: ", RCW);
	SmartDashboard::PutNumber("GRO: ", GRO);

	SmartDashboard::PutNumber("A: ", a);
	SmartDashboard::PutNumber("B: ", b);
	SmartDashboard::PutNumber("C: ", c);
	SmartDashboard::PutNumber("D: ", d);

	SmartDashboard::PutNumber("Front Right Angle: ", fra);
	SmartDashboard::PutNumber("Front Right Steer: ", frs);

	SmartDashboard::PutNumber("Front Left Angle: ", fla);
	SmartDashboard::PutNumber("Front Left Steer: ", fls);

	SmartDashboard::PutNumber("Back Right Angle: ", rra);
	SmartDashboard::PutNumber("Back Right Steer: ", rrs);

	SmartDashboard::PutNumber("Back Left Angle: ", rla);
	SmartDashboard::PutNumber("Back Left Steer: ", rls);
}

void SwerveDrive::Dispose() {
	FRSteer->Disable();
}
