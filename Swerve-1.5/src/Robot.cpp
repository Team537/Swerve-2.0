#include <WPILib.h>
#include <Schematic.h>
#include <Swerve/Swerve.h>

class Robot: public IterativeRobot {
private:
	LiveWindow *window;
	Joystick *joystick;
	Gyro *gyro;
	Swerve *swerve;

	void RobotInit() {
		window = LiveWindow::GetInstance();
		joystick = new Joystick(JOYSTICK_PORT);
		gyro = new Gyro(GYRO_PORT);
		swerve = new Swerve(joystick, gyro);
	}

	void AutonomousInit() {
	}

	void AutonomousPeriodic() {
	}

	void TestInit() {
	}

	void TestPeriodic() {
	}

	void TeleopInit() {
		gyro->InitGyro();
		swerve->Initialize();
	}

	void TeleopPeriodic() {
		if (TUNING_PIDS) {
			swerve->Tune();
		} else {
			swerve->Run();
		}
	}
};

START_ROBOT_CLASS(Robot);
