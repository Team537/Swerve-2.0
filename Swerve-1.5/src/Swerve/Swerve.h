#ifndef SWERVE_SWERVE_H_
#define SWERVE_SWERVE_H_

#include <WPILib.h>
#include <Schematic.h>
#include <Swerve/PIDDistance.h>
#include <Swerve/PIDDriveValue.h>
#include <Swerve/PIDValue.h>
#include <Swerve/SwerveModule.h>

class Swerve {
private:
	SwerveModule *FLModule, *FRModule, *BLModule, *BRModule;
	Joystick *joystick;
	Gyro *gyro;
	Timer *dashboardTimer, *tuningTimer;
	float tuningSetpoint, tuningP, tuningI, tuningD;
	bool initalized;

public:
	Swerve(Joystick *joystick, Gyro *gyro) {
		this->joystick = joystick;
		this->gyro = gyro;
		this->dashboardTimer = new Timer();
		this->tuningTimer = new Timer();
		this->initalized = false;

		this->FLModule = new SwerveModule(FRONT_LEFT_PORTS, new PIDValue(PID_FRONT_LEFT_ANGLE), new PIDDriveValue(PID_FRONT_LEFT_DRIVE), new PIDDistance(PID_FRONT_LEFT_DISTANCE), 0, "Front Left");
		this->FRModule = new SwerveModule(FRONT_RIGHT_PORTS, new PIDValue(PID_FRONT_RIGHT_ANGLE), new PIDDriveValue(PID_FRONT_RIGHT_DRIVE), new PIDDistance(PID_FRONT_RIGHT_DISTANCE), 0, "Front Right");
		this->BLModule = new SwerveModule(BACK_LEFT_PORTS, new PIDValue(PID_BACK_LEFT_ANGLE), new PIDDriveValue(PID_BACK_LEFT_DRIVE), new PIDDistance(PID_BACK_LEFT_DISTANCE), 0, "Back Left");
		this->BRModule = new SwerveModule(BACK_RIGHT_PORTS, new PIDValue(PID_BACK_RIGHT_ANGLE), new PIDDriveValue(PID_BACK_RIGHT_DRIVE), new PIDDistance(PID_BACK_RIGHT_DISTANCE), 0, "Back Right");

		this->tuningSetpoint = 0;
		this->tuningP = 0.0200;
		this->tuningI = 0.0000;
		this->tuningD = 0.0000;
	}

	void Initialize();
	void Run();
	void Tune();
	void AutonomousDrive(float frontLeftAngle, float frontLeftSpeed, float frontRightAngle, float frontRightSpeed, float backLeftAngle, float backLeftSpeed, float backRightAngle, float backRightSpeed);
	void AutonomousDistance(float frontLeft, float frontRight, float backLeft, float backRight);
	bool OnAngleTarget();
	bool OnDistanceTarget();
};

#endif
