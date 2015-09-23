package org.swerve;

import org.Schematic;
import org.components.IComponent;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveComponent extends IComponent {
	private TalonManager FLDrive, FRDrive, BLDrive, BRDrive;
	private TalonManager FLSteer, FRSteer, BLSteer, BRSteer;
	private Gyro gyro;

	/**
	 * Creates a new Swerve Drive (implements {@link org.components.IComponent}).
	 * 
	 * @param fieldOrientated Should this Drive be driven using Field Orientation?
	 * @param robotLength The length of the robot (use same unit of measurement as the width).
	 * @param robotWidth The width of the robot (use same unit of measurement as the length).
	 * @param gyro The gyroscope to be used when orienting the robot.
	 */
	public SwerveComponent(Gyro gyro) {
		this.gyro = gyro;

		this.FLDrive = new TalonManager(0, TypeTalon.SPEED, TypeReadings.ANALOG_ENCODER);
		this.FRDrive = new TalonManager(4, TypeTalon.SPEED, TypeReadings.ANALOG_ENCODER);
		this.BLDrive = new TalonManager(2, TypeTalon.SPEED, TypeReadings.ANALOG_ENCODER);
		this.BRDrive = new TalonManager(6, TypeTalon.SPEED, TypeReadings.ANALOG_ENCODER);

		this.FLSteer = new TalonManager(1, TypeTalon.ANGLE, TypeReadings.POT);
		this.FRSteer = new TalonManager(5, TypeTalon.ANGLE, TypeReadings.POT);
		this.BLSteer = new TalonManager(3, TypeTalon.ANGLE, TypeReadings.POT);
		this.BRSteer = new TalonManager(7, TypeTalon.ANGLE, TypeReadings.POT);
	}

	@Override
	public void update(Joystick joystick1, Joystick joystick2) {
		double FWD = joystick1.getY();
		double STR = joystick1.getX();
		double RCW = joystick1.getZ();
		double GRO = Schematic.FIELD_ORIENTATED ? gyro.getAngle() : 0;

		double FWD2 = (FWD * Math.cos(GRO)) + STR * Math.sin(GRO);
		double STR2 = (-FWD * Math.sin(GRO)) + STR * Math.cos(GRO);

		double r = Math.sqrt((Schematic.ROBOT_LENGTH * Schematic.ROBOT_LENGTH) + (Schematic.ROBOT_WIDTH * Schematic.ROBOT_WIDTH));
		double a = STR2 - RCW * (Schematic.ROBOT_LENGTH / r);
		double b = STR2 + RCW * (Schematic.ROBOT_LENGTH / r);
		double c = FWD2 - RCW * (Schematic.ROBOT_WIDTH / r);
		double d = FWD2 + RCW * (Schematic.ROBOT_WIDTH / r);

		double frs = Math.sqrt(b * b + c * c);
		double fls = Math.sqrt(b * b + d * d);
		double bls = Math.sqrt(a * a + d * d);
		double brs = Math.sqrt(a * a + c * c);
		double fra = Math.atan2(b, c) * 180 / Math.PI;
		double fla = Math.atan2(b, d) * 180 / Math.PI;
		double bra = Math.atan2(a, d) * 180 / Math.PI;
		double bla = Math.atan2(a, c) * 180 / Math.PI;

		double max = frs;

		if (fls > max) {
			max = fls;
		}

		if (bls > max) {
			max = bls;
		}

		if (brs > max) {
			max = brs;
		}

		if (max > 1) {
			frs /= max;
			fls /= max;
			brs /= max;
			bls /= max;
		}

		FRSteer.set((fra + 180) * (1023 / 360));
		FLSteer.set((fla + 180) * (1023 / 360));
		BLSteer.set((bla + 180) * (1023 / 360));
		BRSteer.set((bra + 180) * (1023 / 360));

		FRDrive.set(frs);
		FLDrive.set(fls);
		BRDrive.set(brs);
		BLDrive.set(bls);

		SmartDashboard.putNumber("FWD: ", FWD);
		SmartDashboard.putNumber("STR: ", STR);
		SmartDashboard.putNumber("RCW: ", RCW);
		SmartDashboard.putNumber("GRO: ", GRO);

		SmartDashboard.putNumber("A: ", a);
		SmartDashboard.putNumber("B: ", b);
		SmartDashboard.putNumber("C: ", c);
		SmartDashboard.putNumber("D: ", d);

		SmartDashboard.putNumber("Front Right Angle: ", fra);
		SmartDashboard.putNumber("Front Right Steer: ", frs);
		SmartDashboard.putNumber("Front Left Angle: ", fla);
		SmartDashboard.putNumber("Front Left Steer: ", fls);
		SmartDashboard.putNumber("Back Right Angle: ", bra);
		SmartDashboard.putNumber("Back Right Steer: ", brs);
		SmartDashboard.putNumber("Back Left Angle: ", bla);
		SmartDashboard.putNumber("Back Left Steer: ", bls);
	}
}
