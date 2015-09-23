package org.swerve;

import org.Schematic;
import org.components.IComponent;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive extends IComponent {
	private SwerveModule FLModule, FRModule, BLModule, BRModule;
	private Gyro gyro;
	private Timer dashboardTimer, tuningTimer;
	private float tuningSetpoint, tuningP, tuningI, tuningD;

	public SwerveDrive(Gyro gyro) {
		this.FLModule = new SwerveModule(11, 15, 16, 17, 5, Schematic.PID_FRONT_LEFT_ANGLE, Schematic.PID_FRONT_LEFT_DRIVE, Schematic.PID_FRONT_LEFT_DISTANCE, 0, "Front Left");
		this.FRModule = new SwerveModule(12, 16, 23, 22, 6, Schematic.PID_FRONT_RIGHT_ANGLE, Schematic.PID_FRONT_RIGHT_DRIVE, Schematic.PID_FRONT_RIGHT_DISTANCE, 0, "Front Right");
		this.BLModule = new SwerveModule(10, 14, 14, 15, 4, Schematic.PID_BACK_LEFT_ANGLE, Schematic.PID_BACK_LEFT_DRIVE, Schematic.PID_BACK_LEFT_DISTANCE, 0, "Back Left");
		this.BRModule = new SwerveModule(13, 17, 24, 25, 7, Schematic.PID_BACK_RIGHT_ANGLE, Schematic.PID_BACK_RIGHT_DRIVE, Schematic.PID_BACK_RIGHT_DISTANCE, 0, "Back Right");

		this.gyro = gyro;
		this.dashboardTimer = new Timer();
		this.tuningTimer = new Timer();

		this.tuningSetpoint = 0;
		this.tuningP = 0.0200f;
		this.tuningI = 0.0000f;
		this.tuningD = 0.0000f;

		tuningTimer.start();
		dashboardTimer.start();
	}

	@Override
	public void update(Joystick joystick1, Joystick joystick2) {
		double FWD = -joystick1.getRawAxis(1);
		double STR = -joystick1.getRawAxis(0);
		double RCW = joystick1.getRawAxis(2);
		double GRO = Schematic.FIELD_ORIENTATED ? gyro.getAngle() : 0;

		double FWD2 = (FWD * Math.cos(GRO)) + STR * Math.sin(GRO);
		double STR2 = (-FWD * Math.sin(GRO)) + STR * Math.cos(GRO);

		double r = Math.sqrt((Schematic.ROBOT_WIDTH * Schematic.ROBOT_WIDTH) + (Schematic.ROBOT_LENGTH * Schematic.ROBOT_LENGTH));
		double a = (STR2 - RCW) * (Schematic.ROBOT_WIDTH / r);
		double b = (STR2 + RCW) * (Schematic.ROBOT_WIDTH / r);
		double c = (FWD2 - RCW) * (Schematic.ROBOT_LENGTH / r);
		double d = (FWD2 + RCW) * (Schematic.ROBOT_LENGTH / r);

		double frs = Math.sqrt(b * b + c * c);
		double fls = Math.sqrt(b * b + d * d);
		double bls = Math.sqrt(a * a + d * d);
		double brs = Math.sqrt(a * a + c * c);

		double fra = ((Math.atan2(b, c) * 180 / Math.PI) + 180) * (1023 / 360);
		double fla = ((Math.atan2(b, d) * 180 / Math.PI) + 180) * (1023 / 360);
		double bra = ((Math.atan2(a, d) * 180 / Math.PI) + 180) * (1023 / 360);
		double bla = ((Math.atan2(a, c) * 180 / Math.PI) + 180) * (1023 / 360);

		double max = frs;
		max = (fls > max) ? fls : max;
		max = (bls > max) ? bls : max;
		max = (brs > max) ? brs : max;

		if (max > 1) {
			frs /= max;
			fls /= max;
			brs /= max;
			bls /= max;
		}

		FRModule.steer(fra);
		FLModule.steer(fla);
		BRModule.steer(bra);
		BLModule.steer(bla);

		FRModule.drive(frs);
		FLModule.drive(fls);
		BRModule.drive(bls);
		BLModule.drive(brs);

		dashboardTimer.start();

		if (dashboardTimer.get() > 0.20) {
			FRModule.DashboardPrint();
			FLModule.DashboardPrint();
			BRModule.DashboardPrint();
			BLModule.DashboardPrint();

			SmartDashboard.putNumber("FWD", FWD);
			SmartDashboard.putNumber("STR", STR);
			SmartDashboard.putNumber("RCW", RCW);
			SmartDashboard.putNumber("GRO", GRO);

			SmartDashboard.putNumber("Front Right Angle", fra);
			SmartDashboard.putNumber("Front Right Steer", frs);
			SmartDashboard.putNumber("Front Left Angle", fla);
			SmartDashboard.putNumber("Front Left Steer", fls);
			SmartDashboard.putNumber("Back Right Angle", bra);
			SmartDashboard.putNumber("Back Right Steer", brs);
			SmartDashboard.putNumber("Back Left Angle", bla);
			SmartDashboard.putNumber("Back Left Steer", bls);

			dashboardTimer.stop();
			dashboardTimer.reset();
		}
	}

	public void tune(Joystick joystick) {
		if (joystick.getRawButton(1)) {
			tuningSetpoint = 250;
		} else if (joystick.getRawButton(2)) {
			tuningSetpoint = 185;
		} else {
			tuningSetpoint = 165;
		}

		if (tuningTimer.get() >= 0.10) {
			if (joystick.getRawButton(7)) {
				tuningP -= 0.0001;
				BRModule.PIDResetAngle();
			} else if (joystick.getRawButton(8)) {
				tuningP += 0.0001;
				BRModule.PIDResetAngle();
			}

			if (joystick.getRawButton(3)) {
				tuningI -= 0.00001;
				BRModule.PIDResetAngle();
			} else if (joystick.getRawButton(5)) {
				tuningI += 0.00001;
				BRModule.PIDResetAngle();
			}

			if (joystick.getRawButton(11)) {
				tuningD -= 0.0001;
				BRModule.PIDResetAngle();
			} else if (joystick.getRawButton(12)) {
				tuningD += 0.0001;
				BRModule.PIDResetAngle();
			}

			tuningTimer.stop();
			tuningTimer.reset();
			tuningTimer.start();
			BRModule.steer(tuningSetpoint);
			BRModule.PIDSetAngle(tuningP, tuningI, tuningD);
		}
	}

	public void autonomousDrive(float frontLeftSpeed, float frontRightSpeed, float backLeftSpeed, float backRightSpeed) {
		FLModule.drive(frontLeftSpeed);
		FRModule.drive(frontRightSpeed);
		BLModule.drive(backRightSpeed);
		BRModule.drive(backLeftSpeed);
	}

	public void autonomousSteer(float frontLeftAngle, float frontRightAngle, float backLeftAngle, float backRightAngle) {
		FLModule.steer(frontLeftAngle);
		FRModule.steer(frontRightAngle);
		BLModule.steer(backRightAngle);
		BRModule.steer(backLeftAngle);
	}

	public void autonomousDistance(float frontLeftDistance, float frontRightDistance, float backLeftDistance, float backRightDistance) {
		FLModule.distance(frontLeftDistance);
		FRModule.distance(frontRightDistance);
		BLModule.distance(backLeftDistance);
		BRModule.distance(backRightDistance);
	}

	public boolean onTargetAngle() {
		return (FLModule.PIDAtTargetAngle() && FRModule.PIDAtTargetAngle() && BLModule.PIDAtTargetAngle() && BRModule.PIDAtTargetAngle()) ? true : false;
	}

	public boolean onTargetDistance() {
		return (FLModule.PIDAtTargetDistance() && FRModule.PIDAtTargetDistance() && BLModule.PIDAtTargetDistance() && BRModule.PIDAtTargetDistance()) ? true : false;
	}
}
