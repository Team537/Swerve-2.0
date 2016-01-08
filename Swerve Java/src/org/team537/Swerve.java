package org.team537;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve {
	private SwerveModule frontRight, frontLeft, backLeft, backRight;
	private boolean tuningButton;

	public Swerve() {
		frontLeft = new SwerveModule("Front Right", 9, 6);
		frontRight = new SwerveModule("Front Left", 8, 3);
		backRight = new SwerveModule("Back Left", 4, 1);
		backLeft = new SwerveModule("Back Right", 5, 2);
		tuningButton = false;
	}
	
	public void drive() {
		double fwd = Schematic.deadband(Schematic.JOYSTICK_DEADBAND, Schematic.JOYSTICK_MAIN.getY());
		double str = Schematic.deadband(Schematic.JOYSTICK_DEADBAND, Schematic.JOYSTICK_MAIN.getX());
		double rcw = Schematic.deadband(Schematic.JOYSTICK_DEADBAND, Schematic.JOYSTICK_MAIN.getZ());
		double gro = Schematic.GYROSCOPE.getAngle();

		double fwd2 = (fwd * Math.cos(gro)) + str * Math.sin(gro);
		double str2 = (-fwd * Math.sin(gro)) + str * Math.cos(gro);

		double r = Math.sqrt(Schematic.WHEEL_BASE * Schematic.WHEEL_BASE + Schematic.TRACK_WIDTH * Schematic.TRACK_WIDTH) / 2;
		double a = str2 - rcw * ((Schematic.WHEEL_BASE / r) * 0.5);
		double b = str2 + rcw * ((Schematic.WHEEL_BASE / r) * 0.5);
		double c = fwd2 - rcw * ((Schematic.TRACK_WIDTH / r) * 0.5);
		double d = fwd2 + rcw * ((Schematic.TRACK_WIDTH / r) * 0.5);

		SmartDashboard.putNumber("Inputs Forward", fwd);
		SmartDashboard.putNumber("Inputs Strafe", str);
		SmartDashboard.putNumber("Inputs Rotation", rcw);
		SmartDashboard.putNumber("Inputs Gyro", gro);

		SmartDashboard.putNumber("Maths R", r);
		SmartDashboard.putNumber("Maths A", a);
		SmartDashboard.putNumber("Maths B", b);
		SmartDashboard.putNumber("Maths C", c);
		SmartDashboard.putNumber("Maths D", d);

		double frs = Math.sqrt(b * b + c * c);
		double fls = Math.sqrt(a * a + c * c);
		double bls = Math.sqrt(a * a + d * d);
		double brs = Math.sqrt(b * b + d * d);
		double max = Schematic.maxValue(frs, fls, bls, brs);

		double fra = Math.atan2(b, c) * 180 / Math.PI;
		double fla = Math.atan2(a, c) * 180 / Math.PI;
		double bla = Math.atan2(a, d) * 180 / Math.PI;
		double bra = Math.atan2(b, d) * 180 / Math.PI;

		if (max > 1) {
			frs /= max;
			fls /= max;
			bls /= max;
			brs /= max;
		}

		frontRight.drive(frs, fra);
		frontLeft.drive(fls, fla);
		backLeft.drive(bls, bla);
		backRight.drive(brs, bra);
	}

	public void tune(SwerveModule module) {
		CANTalon talon = module.getAngleTalon();
		boolean stillDown = tuningButton && (tuningButton = Schematic.JOYSTICK_MAIN.getRawButton(10));

		if (tuningButton == !stillDown) {
			if (talon.isControlEnabled()) {
				talon.disable();
			} else {
				talon.enableControl();
			}
		}

		SmartDashboard.putNumber("Tuning Error", talon.getSetpoint() - talon.getAnalogInRaw());
		SmartDashboard.putNumber("Tuning Setpoint", talon.getSetpoint());
		SmartDashboard.putBoolean("Tuning Enabled", talon.isControlEnabled());
	}

	public void dashboard() {
		frontRight.dashboard();
		frontLeft.dashboard();
		backLeft.dashboard();
		backRight.dashboard();
	}
}
