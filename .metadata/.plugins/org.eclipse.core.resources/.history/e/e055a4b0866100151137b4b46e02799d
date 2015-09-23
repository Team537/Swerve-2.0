package org.swerve;

import org.Schematic;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
	private PIDController anglePID, drivePID, distancePID;
	private CANTalon outputSpeed, outputAngle;
	private Encoder speedEncoder;
	private DoublePotentiometer anglePOT;
	private String moduleName;
	private float angleOffset;

	public SwerveModule(int speedPort, int anglePort, int encoder1Port, int encoder2Port, int potPort, PIDValue PIDAngleValues, PIDDriveValue PIDDriveValues, PIDDistance PIDDistanceValues, float angleOffset, String name) {
		this.outputSpeed = new CANTalon(speedPort);
		this.outputAngle = new CANTalon(anglePort);
		this.speedEncoder = new Encoder(encoder1Port, encoder2Port, false, Encoder.EncodingType.k4X);
		this.speedEncoder.setDistancePerPulse(0.023271056296296);
		this.anglePOT = new DoublePotentiometer(potPort, 360, 0, PIDAngleValues.getInputMin(), PIDAngleValues.getInputMax(), name);

		this.moduleName = name;
		this.angleOffset = angleOffset;

		this.anglePID = new PIDController(PIDAngleValues.getP(), PIDAngleValues.getI(), PIDAngleValues.getD(), anglePOT, outputAngle, 0.025);
		this.drivePID = new PIDController(PIDDriveValues.getP(), PIDDriveValues.getI(), PIDDriveValues.getD(), speedEncoder, outputSpeed);
		this.distancePID = new PIDController(PIDDistanceValues.getP(), PIDDistanceValues.getI(), PIDDistanceValues.getD(), speedEncoder, outputSpeed);

		distancePID.setAbsoluteTolerance(10);
		drivePID.setPID(PIDDriveValues.getP(), PIDDriveValues.getI(), PIDDriveValues.getD(), PIDDriveValues.getF());
		anglePID.setInputRange(PIDAngleValues.getInputMin(), PIDAngleValues.getInputMax());
		anglePID.setOutputRange(PIDAngleValues.getOutputMin(), PIDAngleValues.getOutputMax());
		anglePID.setAbsoluteTolerance(10);
		anglePID.setContinuous(true);

		speedEncoder.reset();
		distancePID.reset();
		drivePID.reset();
		anglePID.reset();
	}

	public void drive(double speed) {
		speedEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
		outputSpeed.set(speed * Schematic.DRIVE_MULTIPLIER);

		SmartDashboard.putNumber((moduleName + " Speed Drive Input"), speed);
	}

	public void steer(double angle) {
		angle += angleOffset;

		if (angle < 0) {
			angle += 360;
		} else if (angle >= 360) {
			angle -= 360;
		}

		anglePID.enable();
		anglePID.setSetpoint(angle);

		SmartDashboard.putNumber((moduleName + " Angle Drive Input"), angle);
	}

	public void distance(float distance) {
		speedEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
		distancePID.enable();
		distancePID.setSetpoint(distance);

		SmartDashboard.putNumber((moduleName + " Distance Input"), distance);
	}

	public void PIDSetAngle(float p, float i, float d) {
		anglePID.setPID(p, i, d);
	}

	public boolean PIDAtTargetAngle() {
		return anglePID.onTarget();
	}

	public boolean PIDAtTargetDistance() {
		return distancePID.onTarget();
	}

	public void PIDResetAngle() {
		anglePID.reset();
	}

	public void PIDResetDistance() {
		distancePID.reset();
	}

	public void DashboardPrint() {
		SmartDashboard.putNumber((moduleName + " Speed Encoder"), speedEncoder.getRate());

		SmartDashboard.putNumber((moduleName + " POT Reading"), anglePOT.get());
		SmartDashboard.putNumber((moduleName + " POT Angle"), anglePOT.pidGet());
		SmartDashboard.putNumber((moduleName + " POT Average"), anglePOT.getAverage());

		SmartDashboard.putNumber((moduleName + " Angle Setpoint"), anglePID.getSetpoint());
		SmartDashboard.putBoolean((moduleName + " Angle On Target"), anglePID.onTarget());
		SmartDashboard.putNumber((moduleName + " Angle Error"), anglePID.getError());
		// SmartDashboard.putNumber((moduleName + " Angle P"), anglePID.getP());
		// SmartDashboard.putNumber((moduleName + " Angle I"), anglePID.getI());
		// SmartDashboard.putNumber((moduleName + " Angle D"), anglePID.getD());

		SmartDashboard.putNumber((moduleName + " Distance Setpoint"), distancePID.getSetpoint());
		SmartDashboard.putBoolean((moduleName + " Distance On Target"), distancePID.onTarget());
		SmartDashboard.putNumber((moduleName + " Distance Error"), distancePID.getError());
		// SmartDashboard.putNumber((moduleName + " Distance P"), distancePID.getP());
		// SmartDashboard.putNumber((moduleName + " Distance I"), distancePID.getI());
		// SmartDashboard.putNumber((moduleName + " Distance D"), distancePID.getD());
	}
}
