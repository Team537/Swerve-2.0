#include "Schematic.hpp"
#include "Swerve.hpp"

class Robot: public IterativeRobot {
private:
	Joystick *joystick;
	Gyro *gyro;
	Swerve *swerve;

	void RobotInit() {
		joystick = new Joystick(0);
		gyro = new Gyro(1);
		gyro->Reset();
		swerve = new Swerve(joystick, gyro);
	}

	void AutonomousInit() {
	}

	void AutonomousPeriodic() {
	}

	void TeleopInit() {
	}

	void TeleopPeriodic() {
		swerve->Drive();
		// swerve->Tune(swerve->frontRightAngle);
		// swerve->TuneMinMax(swerve->frontRightAngle);
		swerve->Dashboard();
	}

	void TestPeriodic() {
	}
};

START_ROBOT_CLASS(Robot);
