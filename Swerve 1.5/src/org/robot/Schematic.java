package org.robot;

import org.robot.swerve.PIDValue;

public class Schematic {
	public static final boolean FIELD_ORIENTATED = false;

	public static final int JOYSTICK_MAIN = 0;
	public static final int JOYSTICK_SECONDARY = 1;

	public static final double DRIVE_MULTIPLIER = 0.6;
	public static final double ROBOT_LENGTH = 22;
	public static final double ROBOT_WIDTH = 22;

	public static final int PORT_GYRO = 0;

	public static final PIDValue PID_FRONT_LEFT_ANGLE = new PIDValue(0.030f, -0.000290f, 0.050f, 0, 360, -1, 1);
	public static final PIDValue PID_FRONT_RIGHT_ANGLE = new PIDValue(0.030f, 0.000290f, 0.050f, 0, 360, -1, 1);
	public static final PIDValue PID_BACK_RIGHT_ANGLE = new PIDValue(6.4f / 1000, 0.0f, 1.1f / 1000, 0, 360, -1, 1);
	public static final PIDValue PID_BACK_LEFT_ANGLE = new PIDValue(0.030f, 0.000290f, 0.050f, 0, 360, -1, 1);
}
