#ifndef SWERVE_SWERVEMODULE_H_
#define SWERVE_SWERVEMODULE_H_

#include <cmath>
#include <ctime>
#include <string>
#include <WPIlib.h>
#include <Schematic.h>
#include <Swerve/DoublePot.h>
#include <Swerve/PIDDistance.h>
#include <Swerve/PIDDriveValue.h>
#include <Swerve/PIDValue.h>

class SwerveModule {
private:
	PIDController *anglePID, *drivePID, *distancePID;
	CANTalon *outputSpeed, *outputAngle;
	Encoder *speedEncoder;
	DoublePot *anglePOT;
	Timer *dashboardTimer;
	std::string moduleName;
	float angleOffset;

public:
	SwerveModule(int speedPort, int anglePort, int encoder1Port, int encoder2Port, int potPort, PIDValue *PIDAngleValues, PIDDriveValue *PIDDriveValues, PIDDistance *PIDDistanceValues, float angleOffset, std::string name) {
		this->outputSpeed = new CANTalon(speedPort);
		this->outputAngle = new CANTalon(anglePort);
		this->speedEncoder = new Encoder(encoder1Port, encoder2Port, false, Encoder::EncodingType::k4X);
		this->speedEncoder->SetDistancePerPulse(0.023271056296296);
		this->anglePOT = new DoublePot(potPort, 360, PIDAngleValues->getInputMin(), PIDAngleValues->getInputMax(), 0, name);
		this->dashboardTimer = new Timer();

		this->moduleName = name;
		this->angleOffset = angleOffset;

		this->anglePID = new PIDController(PIDAngleValues->getP(), PIDAngleValues->getI(), PIDAngleValues->getD(), anglePOT, outputAngle, 0.025);
		this->drivePID = new PIDController(PIDDriveValues->getP(), PIDDriveValues->getI(), PIDDriveValues->getD(), speedEncoder, outputSpeed);
		this->distancePID = new PIDController(PIDDistanceValues->getP(), PIDDistanceValues->getI(), PIDDistanceValues->getD(), speedEncoder, outputSpeed);

		distancePID->SetTolerance(5);
		drivePID->SetPID(PIDDriveValues->getP(), PIDDriveValues->getI(), PIDDriveValues->getD(), PIDDriveValues->getF());
		anglePID->SetInputRange(PIDAngleValues->getInputMin(), PIDAngleValues->getInputMax());
		anglePID->SetOutputRange(PIDAngleValues->getOutputMin(), PIDAngleValues->getOutputMax());
		anglePID->SetAbsoluteTolerance(20);
		anglePID->SetContinuous(true);
	}

	void Initialize();
	void Drive(float angle, float speed);
	void DriveAuto(float distance);
	void PIDAdjustAngle(float p, float i, float d);
	float POTReadAngle();
	bool PIDAtTargetAngle();
	bool PIDAtTargetDistance();
	void PIDResetAngle();
	void PIDResetDistance();
	void PIDDisableAngle();
	void PIDDisableDistance();
	void DashboardLoop();
};

#endif
