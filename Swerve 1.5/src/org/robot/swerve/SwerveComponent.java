package org.robot.swerve;

import org.robot.Schematic;
import org.robot.components.IComponent;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveComponent extends IComponent {
	private DriveModule /* FLModule, */FRModule;// , BLModule, BRModule;
	private Gyro gyro;

	private boolean tuning = true;

	public SwerveComponent(Gyro gyro) {
		// this.FLModule = new DriveModule(8, 3, 2, Schematic.PID_FRONT_LEFT_ANGLE, 0, "Front Left");
		this.FRModule = new DriveModule(9, 6, 1, Schematic.PID_FRONT_RIGHT_ANGLE, 0, "Front Right");
		// this.BLModule = new DriveModule(4, 1, 4, Schematic.PID_BACK_LEFT_ANGLE, 0, "Back Left");
		// this.BRModule = new DriveModule(5, 2, 3, Schematic.PID_BACK_RIGHT_ANGLE, 0, "Back Right");
		this.gyro = gyro;
		lastTime = System.currentTimeMillis();
	}

	@Override
	public void update(Joystick joystick1, Joystick joystick2) {
		if (joystick1.getRawButton(12)) {
			tuning = !tuning;
		}
		
		if (tuning) {
			tune(joystick1, FRModule);
			return;
		}

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

		double fra = ((Math.atan2(b, c) * 180 / Math.PI) + 180);
		double fla = ((Math.atan2(b, d) * 180 / Math.PI) + 180);
		double bra = ((Math.atan2(a, d) * 180 / Math.PI) + 180);
		double bla = ((Math.atan2(a, c) * 180 / Math.PI) + 180);

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

		SmartDashboard.putNumber("FRA MATH", fra);

		FRModule.steer(fra);
		// FLModule.steer(fla);
		// BRModule.steer(bra);
		// BLModule.steer(bla);

		// FRModule.drive(frs * Schematic.DRIVE_MULTIPLIER);
		// FLModule.drive(fls * Schematic.DRIVE_MULTIPLIER);
		// BRModule.drive(bls * Schematic.DRIVE_MULTIPLIER);
		// BLModule.drive(brs * Schematic.DRIVE_MULTIPLIER);
		dashboardUpdate();
	}

	private float tuningP, tuningI, tuningD;
	private long lastTime;

	public void tune(Joystick joystick, DriveModule module) {
		long passedTime = System.currentTimeMillis() - lastTime;

		if (passedTime > 100) {
			if (joystick.getRawButton(5)) { // P up
				tuningP += 0.0001;
			} else if (joystick.getRawButton(7)) { // P down
				tuningP -= 0.0001;
			}

			if (joystick.getRawButton(3)) { // P up 'B'
				tuningI += 0.00001;
			} else if (joystick.getRawButton(1)) { // P down 'X'
				tuningI -= 0.00001;
			}

			if (joystick.getRawButton(6)) { // D up
				tuningD += 0.0001;
			} else if (joystick.getRawButton(8)) { // D down
				tuningD -= 0.0001;
			}

			float newSetPoint = 0;

			if (joystick.getRawButton(10)) { // Setpoint 180 (start)
				newSetPoint = 180;
			} else if (joystick.getRawButton(9)) { // Setpoint 0 (back)
				newSetPoint = 0;
			}

			module.getAnglePID().setPID(tuningP, tuningI, tuningD);
			module.getAnglePID().setSetpoint(newSetPoint);

			SmartDashboard.putNumber("TuningP", tuningP * 1000);
			SmartDashboard.putNumber("TuningI", tuningI * 10000);
			SmartDashboard.putNumber("TuningD", tuningD * 1000);
			SmartDashboard.putNumber("POT Position", module.getAnglePID().get());
			SmartDashboard.putNumber("POT Error", module.getAnglePID().getError());
			SmartDashboard.putNumber("POT Setpoint", module.getAnglePID().getSetpoint());
			lastTime = System.currentTimeMillis();
		}
	}

	private void dashboardUpdate() {
		FRModule.dashboardPrint();
		// FLModule.dashboardPrint();
		// BRModule.dashboardPrint();
		// BLModule.dashboardPrint();
	}

	public boolean onTargetAngle() {
		return false;
		// return (FLModule.PIDAtTargetAngle() && FRModule.PIDAtTargetAngle() && BLModule.PIDAtTargetAngle() && BRModule.PIDAtTargetAngle()) ? true : false;
	}
}
