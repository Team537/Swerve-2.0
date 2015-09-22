#ifndef SCHEMATIC_H_
#define SCHEMATIC_H_

// Math constants.
#define PI 3.1415926535

// Are you tuning the robot's PID's?
#define TUNING_PIDS false

// Used when driving the robot, how much speed will be take off this is always set so the motors don't work at 100% and burn out.
#define DRIVE_MULTIPLIER 0.75
#define FIELD_ORIENTATED false

// Other robot related values.
#define JOYSTICK_PORT 0
#define ROBOT_LENGTH 36.5
#define ROBOT_WIDTH 22

// More random robot ports.
#define GYRO_PORT 0

// Ports to be used in each swerve module, order (speedPort, anglePort, encoder1Port, encoder2Port, potPort)
#define FRONT_LEFT_PORTS 11, 15, 16, 17, 5
#define FRONT_RIGHT_PORTS 12, 16, 23, 22, 6
#define BACK_LEFT_PORTS 10, 14, 14, 15, 4
#define BACK_RIGHT_PORTS 13, 17, 24, 25, 7

// Drive train Angle PID's.
#define PID_FRONT_LEFT_ANGLE 0.030, -0.000290, 0.050, 15, 345, -1, 1
#define PID_FRONT_RIGHT_ANGLE 0.030, 0.000290, 0.050, 15, 345, -1, 1
#define PID_BACK_RIGHT_ANGLE 0.030, 0.000290, 0.050, 15, 345, -1, 1
#define PID_BACK_LEFT_ANGLE 0.030, 0.000290, 0.050, 15, 345, -1, 1

// Drive train Drive PID's.
#define PID_FRONT_LEFT_DRIVE 0.004, 0, 0, 0, (0.5 / 450.0)
#define PID_FRONT_RIGHT_DRIVE 0.004, 0, 0, 0, (0.5 / 450.0)
#define PID_BACK_LEFT_DRIVE 0.004, 0, 0, 0, (0.5 / 450.0)
#define PID_BACK_RIGHT_DRIVE 0.004, 0, 0, 0, (0.5 / 450.0)

// Drive train Distance PID's.
#define PID_FRONT_LEFT_DISTANCE	0.04, 0, 0
#define PID_FRONT_RIGHT_DISTANCE 0.04, 0, 0
#define PID_BACK_LEFT_DISTANCE 0.04, 0, 0
#define PID_BACK_RIGHT_DISTANCE 0.04, 0, 0

#endif
