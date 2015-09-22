package org;

import org.swerve.PIDDistance;
import org.swerve.PIDDriveValue;
import org.swerve.PIDValue;

public class Schematic {
	public static final boolean TUNING_PIDS = false;
	public static final boolean FIELD_ORIENTATED = false;

	public static final int JOYSTICK_MAIN = 0;
	public static final int JOYSTICK_SECONDARY = 1;

	public static final double DRIVE_MULTIPLIER = 0.50;
	public static final double ROBOT_LENGTH = 22;
	public static final double ROBOT_WIDTH = 22;

	public static final int PORT_GYRO = 0;

	public static final PIDValue PID_FRONT_LEFT_ANGLE = new PIDValue(0.030f, -0.000290f, 0.050f, 15f, 345f, -1, 1);
	public static final PIDValue PID_FRONT_RIGHT_ANGLE = new PIDValue(0.030f, 0.000290f, 0.050f, 15f, 345f, -1, 1);
	public static final PIDValue PID_BACK_RIGHT_ANGLE = new PIDValue(0.030f, 0.000290f, 0.050f, 15f, 345f, -1, 1);
	public static final PIDValue PID_BACK_LEFT_ANGLE = new PIDValue(0.030f, 0.000290f, 0.050f, 15f, 345f, -1, 1);

	public static final PIDDriveValue PID_FRONT_LEFT_DRIVE = new PIDDriveValue(0.004f, 0, 0, 0, (0.5f / 450.0f));
	public static final PIDDriveValue PID_FRONT_RIGHT_DRIVE = new PIDDriveValue(0.004f, 0, 0, 0, (0.5f / 450.0f));
	public static final PIDDriveValue PID_BACK_LEFT_DRIVE = new PIDDriveValue(0.004f, 0, 0, 0, (0.5f / 450.0f));
	public static final PIDDriveValue PID_BACK_RIGHT_DRIVE = new PIDDriveValue(0.004f, 0, 0, 0, (0.5f / 450.0f));

	public static final PIDDistance PID_FRONT_LEFT_DISTANCE = new PIDDistance(0.04f, 0, 0);
	public static final PIDDistance PID_FRONT_RIGHT_DISTANCE = new PIDDistance(0.04f, 0, 0);
	public static final PIDDistance PID_BACK_LEFT_DISTANCE = new PIDDistance(0.04f, 0, 0);
	public static final PIDDistance PID_BACK_RIGHT_DISTANCE = new PIDDistance(0.04f, 0, 0);
}
