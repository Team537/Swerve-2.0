#include "Schematic.h"
#include "Swerve/SwerveDrive.h"

class Robot : public SampleRobot {
private:
	Joystick *joystick;
	Gyro *gyro;
	LiveWindow *liveWindow;
	SwerveDrive *drive;

public:
	void RobotInit() {
		joystick = new Joystick(0);
		gyro = new Gyro(0);
		drive = new SwerveDrive(joystick, gyro, liveWindow);
	}

	void AutonomousInit() {
	}

	void AutonomousPeriodic() {
	}

	void TeleopInit() {
		drive->Initalize();
	}

	void TeleopPeriodic() {
		drive->Update();
	}

	void TestInit() {
	}

	void TestPeriodic() {
	}
};

START_ROBOT_CLASS(Robot);
