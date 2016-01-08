package org.team537;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;

public class Schematic {
	/* Robot body parameters (cm). */
	public static final double WHEEL_BASE = 58.0f;
	public static final double TRACK_WIDTH = 58.0f;

	/* Generic robot parameters. */
	public static final double JOYSTICK_DEADBAND = 0.2f;
	public static final Joystick JOYSTICK_MAIN = new Joystick(0);
	public static final Joystick JOYSTICK_SECONDARY = new Joystick(1);
	public static final Gyro GYROSCOPE = new Gyro(0);

	/* Swerve drive parameters. */
	public static final double SPEED_MULTIPLIER = 0.5f;
	public static final boolean DRIVE_ENABLED = true;
	public static final boolean STEER_ENABLED = true;
	
	public static final int[] MODULE_FRONT_RIGHT = new int[] { 9, 6 };
	public static final int[] MODULE_FRONT_LEFT = new int[] { 8, 3 };
	public static final int[] MODULE_BACK_LEFT = new int[] { 4, 1 };
	public static final int[] MODULE_BACK_RIGHT = new int[] { 5, 2 };
	
	public static double maxValue(double... fs) {
		double max = 0;

		for (double v : fs) {
			if (v > max) {
				max = v;
			}
		}

		return max;
	}

	public static double deadband(double min, double value) {
		return Math.abs(value) >= min ? value : 0;
	}
}
