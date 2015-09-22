#include <Swerve/Swerve.h>

void Swerve::Initialize() {
	if (initalized) {
		return;
	}

	FLModule->Initialize();
	FRModule->Initialize();
	BLModule->Initialize();
	BRModule->Initialize();

	tuningTimer->Stop();
	tuningTimer->Reset();
	tuningTimer->Start();
	dashboardTimer->Stop();
	dashboardTimer->Reset();
	dashboardTimer->Start();

	initalized = true;
}

void Swerve::Run() {
	double FWD = -joystick->GetRawAxis(1);
	double STR = -joystick->GetRawAxis(0);
	double RCW = joystick->GetRawAxis(2);
	double GRO = FIELD_ORIENTATED ? gyro->GetAngle() : 0;

	double FWD2 = (FWD * cos(GRO)) + STR * sin(GRO);
	double STR2 = (-FWD * sin(GRO)) + STR * cos(GRO);

	double r = sqrt((ROBOT_WIDTH * ROBOT_WIDTH) + (ROBOT_LENGTH * ROBOT_LENGTH));
	double a = (STR2 - RCW) * (ROBOT_WIDTH / r);
	double b = (STR2 + RCW) * (ROBOT_WIDTH / r);
	double c = (FWD2 - RCW) * (ROBOT_LENGTH / r);
	double d = (FWD2 + RCW) * (ROBOT_LENGTH / r);

	double frs = sqrt(b * b + c * c);
	double fls = sqrt(b * b + d * d);
	double bls = sqrt(a * a + d * d);
	double brs = sqrt(a * a + c * c);

	double fra = ((atan2(b, c) * 180 / PI) + 180) * (1023 / 360);
	double fla = ((atan2(b, d) * 180 / PI) + 180) * (1023 / 360);
	double bra = ((atan2(a, d) * 180 / PI) + 180) * (1023 / 360);
	double bla = ((atan2(a, c) * 180 / PI) + 180) * (1023 / 360);

	double max = frs;
	max = (fls > max) ? fls : max;
	max = (bls > max) ? bls : max;
	max = (brs > max) ? brs : max;

	if (max > 1) {
		frs /= max;
		fls /= max;
		brs /= max;
		bls /= max;
	}

	FRModule->Drive(fra, frs);
	FLModule->Drive(fla, fls);
	BRModule->Drive(bra, bls);
	BLModule->Drive(bla, brs);

	dashboardTimer->Start();

	if (dashboardTimer->Get() > 0.20) {
		SmartDashboard::PutNumber("FWD", FWD);
		SmartDashboard::PutNumber("STR", STR);
		SmartDashboard::PutNumber("RCW", RCW);
		SmartDashboard::PutNumber("GRO", GRO);

		SmartDashboard::PutNumber("Front Right Angle", fra);
		SmartDashboard::PutNumber("Front Right Steer", frs);
		SmartDashboard::PutNumber("Front Left Angle", fla);
		SmartDashboard::PutNumber("Front Left Steer", fls);
		SmartDashboard::PutNumber("Back Right Angle", bra);
		SmartDashboard::PutNumber("Back Right Steer", brs);
		SmartDashboard::PutNumber("Back Left Angle", bla);
		SmartDashboard::PutNumber("Back Left Steer", bls);
		dashboardTimer->Stop();
		dashboardTimer->Reset();
	}
}

void Swerve::Tune() {
	// Get Angle based off Button Inputs.
	if (joystick->GetRawButton(1) == 1) {
		tuningSetpoint = 250;
	} else if (joystick->GetRawButton(2)) {
		tuningSetpoint = 185;
	} else {
		tuningSetpoint = 165;
	}

	// Manually drive the swerve moduals for sensor testing.
	// FRONTLEFTMOD->Drive(CONTROLLER->GetRawAxis(1), 0);
	// FRONTRIGHTMOD->Drive(CONTROLLER->GetRawAxis(1), 0);
	// BACKRIGHTMOD->Drive(CONTROLLER->GetRawAxis(1), 0);
	// BACKLEFTMOD->Drive(CONTROLLER->GetRawAxis(1), 0);

	// Get Angle Based off Joystick.
	// ANGLESETPOINT = Controller->GetDirectionDegrees();
	// ANGLESETPOINT += 180;
	// SmartDashboard::PutNumber("Angle Setpoint", ANGLESETPOINT);

	// Enables AutoStear.
	// FRONTRIGHTMOD->Drive(0, ANGLESETPOINT);
	// BACKLEFTMOD->Drive(0, ANGLESETPOINT);
	// BACKRIGHTMOD->Drive(0, ANGLESETPOINT);
	// FRONTLEFTMOD->Drive(0, ANGLESETPOINT);

	if (tuningTimer->Get() >= 0.10) {
		if (joystick->GetRawButton(7) == 1) {
			tuningP -= 0.0001;
			BRModule->PIDResetAngle();
		} else if (joystick->GetRawButton(8) == 1) {
			tuningP += 0.0001;
			BRModule->PIDResetAngle();
		}

		if (joystick->GetRawButton(3) == 1) {
			tuningI -= 0.00001;
			BRModule->PIDResetAngle();
		} else if (joystick->GetRawButton(5) == 1) {
			tuningI += 0.00001;
			BRModule->PIDResetAngle();
		}

		if (joystick->GetRawButton(11) == 1) {
			tuningD -= 0.0001;
			BRModule->PIDResetAngle();
		} else if (joystick->GetRawButton(12) == 1) {
			tuningD += 0.0001;
			BRModule->PIDResetAngle();
		}

		tuningTimer->Stop();
		tuningTimer->Reset();
		tuningTimer->Start();
		BRModule->Drive(tuningSetpoint, 0);
		BRModule->PIDAdjustAngle(tuningP, tuningI, tuningD);
	}
}

void Swerve::AutonomousDrive(float frontLeftAngle, float frontLeftSpeed, float frontRightAngle, float frontRightSpeed, float backLeftAngle, float backLeftSpeed, float backRightAngle, float backRightSpeed) {
	FLModule->Drive(frontLeftAngle, frontLeftSpeed);
	FRModule->Drive(frontRightAngle, frontRightSpeed);
	BRModule->Drive(backLeftAngle, backLeftSpeed);
	BLModule->Drive(backRightAngle, backRightSpeed);
}

void Swerve::AutonomousDistance(float frontLeft, float frontRight, float backLeft, float backRight) {
	FLModule->DriveAuto(frontLeft);
	FRModule->DriveAuto(frontRight);
	BLModule->DriveAuto(backLeft);
	BRModule->DriveAuto(backRight);
}

bool Swerve::OnAngleTarget() {
	return (FLModule->PIDAtTargetAngle() && FRModule->PIDAtTargetAngle() && BLModule->PIDAtTargetAngle() && BRModule->PIDAtTargetAngle()) ? true : false;
}

bool Swerve::OnDistanceTarget() {
	return (FLModule->PIDAtTargetDistance(), FRModule->PIDAtTargetDistance(), BLModule->PIDAtTargetDistance(), BRModule->PIDAtTargetDistance()) ? true : false;
}
