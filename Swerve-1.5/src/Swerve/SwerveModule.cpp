#include <Swerve/SwerveModule.h>

void SwerveModule::Initialize() {
	anglePID->Disable();
	anglePID->Reset();
	anglePID->Enable();
	speedEncoder->Reset();

	dashboardTimer->Stop();
	dashboardTimer->Reset();
	dashboardTimer->Start();
}

void SwerveModule::Drive(float angle, float speed) {
	angle += angleOffset;

	if (angle < 0) {
		angle += 360;
	} else if (angle >= 360) {
		angle -= 360;
	}

	speedEncoder->SetPIDSourceParameter(PIDSource::kRate);
	anglePID->Enable();
	anglePID->SetSetpoint(angle);

	if (abs(speed) < 0.10) {
		speed = 0;
	}

	outputSpeed->Set(speed * DRIVE_MULTIPLIER);

	SmartDashboard::PutNumber((moduleName + " Angle Drive Input"), angle);
	SmartDashboard::PutNumber((moduleName + " Speed Drive Input"), speed);
}

void SwerveModule::DriveAuto(float distance) {
	speedEncoder->SetPIDSourceParameter(PIDSource::kDistance);
	distancePID->Enable();
	distancePID->SetSetpoint(distance);
}

void SwerveModule::PIDAdjustAngle(float p, float i, float d) {
	anglePID->SetPID(p, i, d);
}

float SwerveModule::POTReadAngle() {
	return anglePOT->PIDGet();
}

bool SwerveModule::PIDAtTargetAngle() {
	return anglePID->OnTarget();
}

bool SwerveModule::PIDAtTargetDistance() {
	return distancePID->OnTarget();
}

void SwerveModule::PIDResetAngle() {
	anglePID->Reset();
}

void SwerveModule::PIDResetDistance() {
	distancePID->Reset();
}

void SwerveModule::PIDDisableAngle() {
	anglePID->Disable();
}

void SwerveModule::PIDDisableDistance() {
	distancePID->Disable();
}

void SwerveModule::DashboardLoop() {
	dashboardTimer->Start();

	if (dashboardTimer->Get() > 0.20) {
		SmartDashboard::PutNumber((moduleName + " Speed Encoder"), speedEncoder->GetRate());
		SmartDashboard::PutNumber((moduleName + " POT"), anglePOT->Get());
		SmartDashboard::PutNumber((moduleName + " POT PID"), anglePOT->PIDGet());
		SmartDashboard::PutNumber((moduleName + " POT Average"), anglePOT->GetAverage());
		SmartDashboard::PutNumber((moduleName + " Angle Setpoint"), anglePID->GetSetpoint());
		SmartDashboard::PutBoolean((moduleName + " Angle On Target"), anglePID->OnTarget());
		SmartDashboard::PutNumber((moduleName + " Angle Error"), anglePID->GetError());
		SmartDashboard::PutNumber((moduleName + " Angle P"), (anglePID->GetP() * 1000));
		SmartDashboard::PutNumber((moduleName + " Angle I"), (anglePID->GetI() * 1000));
		SmartDashboard::PutNumber((moduleName + " Angle D"), (anglePID->GetD() * 1000));
		SmartDashboard::PutBoolean((moduleName + " Distance On Target"), distancePID->OnTarget());
		SmartDashboard::PutNumber((moduleName + " Distance Error"), distancePID->GetError());

		dashboardTimer->Stop();
		dashboardTimer->Reset();
	}
}
