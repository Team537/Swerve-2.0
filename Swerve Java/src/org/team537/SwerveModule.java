package org.team537;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
	private CANTalon driveTalon, angleTalon;
	private String moduleName;
	private float potMin, potMax;
	private int driveSign;

	public SwerveModule(String name, int drivePort, int anglePort) {
		/* Drive Talon */
		driveTalon = new CANTalon(drivePort);
		driveTalon.changeControlMode(CANTalon.ControlMode.PercentVbus);
		driveTalon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);

		/* Angle Talon */
		angleTalon = new CANTalon(anglePort);
		angleTalon.changeControlMode(CANTalon.ControlMode.Position);
		angleTalon.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);

		if (Schematic.DRIVE_ENABLED) {
			driveTalon.enableControl();
		} else {
			driveTalon.disable();
		}

		if (Schematic.STEER_ENABLED) {
			angleTalon.enableControl();
		} else {
			angleTalon.disable();
		}

		/* Module Values. */
		moduleName = name;

		/* POT Values. */
		potMin = 50;
		potMax = 975;
		driveSign = 1;
	}

	private void updatePOT() {
		if (angleTalon.getAnalogInRaw() < potMin) {
			potMin = angleTalon.getAnalogInRaw();
		}

		if (angleTalon.getAnalogInRaw() > potMax) {
			potMax = angleTalon.getAnalogInRaw();
		}
	}

	private double FixAngleValues(double angle) {
		return (angle <= 0) ? angle + 360 : angle;
	}

	private double FixOutputAngle(double angle) {
		if (angle > 330) {
			driveSign = -1;
			angle -= 180;
		} else if (angle < 30) {
			driveSign = -1;
			angle += 180;
		} else {
			driveSign = 1;
		}

		return angle;
	}

	private double calculatePOTValue(double angle) {
		return ((angle / 360) * (potMax - potMin)) + potMin;
	}

	public void drive(double speed, double angle) {
		updatePOT();
		double angleValue = FixOutputAngle(FixAngleValues(angle));
		SmartDashboard.putNumber(moduleName + " Angle calc", angleValue);
		angleTalon.set(calculatePOTValue(angleValue));
		driveTalon.set(driveSign * speed * Schematic.SPEED_MULTIPLIER);
	}

	public void dashboard() {
		SmartDashboard.putNumber(moduleName + " Angle Readings", angleTalon.getAnalogInRaw());
		SmartDashboard.putNumber(moduleName + " Angle Setpoint", angleTalon.getSetpoint());

		SmartDashboard.putNumber(moduleName + " Drive Readings", driveTalon.getAnalogInRaw());
		SmartDashboard.putNumber(moduleName + " Drive Velocity", driveTalon.getAnalogInVelocity());
	}

	public CANTalon getDriveTalon() {
		return driveTalon;
	}

	public CANTalon getAngleTalon() {
		return angleTalon;
	}
	
}
