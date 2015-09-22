#ifndef SWERVE_TALONMANAGER_H_
#define SWERVE_TALONMANAGER_H_

#include "Schematic.h"

enum TypeTalon {
	SPEED, ANGLE, FOLLOWER
};

enum TypeReadings {
	POT, ANALOG_ENCODER, QUAD_ENCODER, NONE
};

class TalonManager {
private:
	CANTalon *Talon;
	TypeTalon talonType;
	TypeReadings talonReadings;
	double OutputCurrent, OutputVoltage, BusVoltage;
	double TalonPosition, TalonVelocity, TalonError;

public:
	TalonManager(int port, TypeTalon type, TypeReadings readings) {
		Talon = new CANTalon(port);

		this->talonType = type;
		this->talonReadings = readings;

		this->OutputCurrent = 0;
		this->OutputVoltage = 0;
		this->BusVoltage = 0;
		this->TalonPosition = 0;
		this->TalonVelocity = 0;
		this->TalonError = 0;

		switch (talonType) {
			case SPEED:
				Talon->SetControlMode(CANSpeedController::kPercentVbus); // Set() controls throttleable speed;
				break;
			case ANGLE:
				Talon->SetControlMode(CANSpeedController::kPosition); // Set() controls setpoint.
				break;
			case FOLLOWER:
				Talon->SetControlMode(CANSpeedController::kFollower); // Set() determines the Talon to follow.
				break;
		}

		switch (talonReadings) {
			case POT:
				Talon->SetFeedbackDevice(CANTalon::AnalogPot);
				break;
			case ANALOG_ENCODER:
				Talon->SetFeedbackDevice(CANTalon::AnalogEncoder);
				break;
			case QUAD_ENCODER:
				Talon->SetFeedbackDevice(CANTalon::QuadEncoder);
				break;
			case NONE:
				break;
		}

		// Set a new target (0 speed || 0 angle).
		Set(0);
	}

	void SetPID(double p, double i, double d) {
		Talon->SetPID(p, i, d);
	}

	void Set(float target) {
		Talon->Set(target);
	}

	void Update() {
		this->OutputCurrent = Talon->GetOutputCurrent();
		this->OutputVoltage = Talon->GetOutputVoltage();
		this->BusVoltage = Talon->GetBusVoltage();

		switch (talonType) {
			case SPEED:
				Talon->GetSpeed();
				break;
			case ANGLE:
				Talon->GetPosition();
				break;
			case FOLLOWER:
				break;
		}

		switch (talonReadings) {
			case POT:
				this->TalonPosition = Talon->GetAnalogIn();
				this->TalonVelocity = Talon->GetAnalogInVel();
				this->TalonError = Talon->GetClosedLoopError();
				break;
			case ANALOG_ENCODER:
				this->TalonPosition = Talon->GetEncPosition();
				this->TalonVelocity = Talon->GetEncVel();
				this->TalonError = Talon->GetClosedLoopError();
				break;
			case QUAD_ENCODER:
				this->TalonPosition = Talon->GetEncPosition();
				this->TalonVelocity = Talon->GetEncVel();
				this->TalonError = Talon->GetClosedLoopError();
				break;
			case NONE:
				this->TalonPosition = -1;
				this->TalonVelocity = -1;
				this->TalonError = -1;
				break;
		}
	}

	void Disable() {
		Talon->Disable();
	}
};

#endif
