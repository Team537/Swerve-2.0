#ifndef SWERVE_SWERVEDRIVE_H_
#define SWERVE_SWERVEDRIVE_H_

#include "Schematic.h"
#include "Swerve/TalonManager.h"

class SwerveDrive {
private:
	TalonManager *FLDrive, *RLDrive, *FRDrive, *RRDrive;
	TalonManager *FLSteer, *RLSteer, *FRSteer, *RRSteer;
	Joystick *joystick;
	Gyro *gyro;
	LiveWindow *liveWindow;

public:
	SwerveDrive(Joystick *joystick, Gyro *gyro, LiveWindow *liveWindow) {
		FLDrive = new TalonManager(0, TypeTalon::SPEED, TypeReadings::ANALOG_ENCODER);
		RLDrive = new TalonManager(2, TypeTalon::SPEED, TypeReadings::ANALOG_ENCODER);
		FRDrive = new TalonManager(4, TypeTalon::SPEED, TypeReadings::ANALOG_ENCODER);
		RRDrive = new TalonManager(6, TypeTalon::SPEED, TypeReadings::ANALOG_ENCODER);

		FLSteer = new TalonManager(1, TypeTalon::ANGLE, TypeReadings::POT);
		RLSteer = new TalonManager(3, TypeTalon::ANGLE, TypeReadings::POT);
		FRSteer = new TalonManager(5, TypeTalon::ANGLE, TypeReadings::POT);
		RRSteer = new TalonManager(7, TypeTalon::ANGLE, TypeReadings::POT);

		FLSteer->SetPID(1, 0, 0);
		RLSteer->SetPID(1, 0, 0);
		FRSteer->SetPID(1, 0, 0);
		RRSteer->SetPID(1, 0, 0);

		this->joystick = joystick;
		this->gyro = gyro;
		this->liveWindow = liveWindow;
	}

	void Initalize();
	void Update();
	void Dispose();
};

#endif
