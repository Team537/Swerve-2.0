package org;

import java.util.ArrayList;
import java.util.List;

import org.components.IComponent;
import org.swerve.SwerveComponent;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;

public class Robot extends SampleRobot {
	private List<IComponent> components;
	private Joystick joystick;
	private Gyro gyro;

	public void robotInit() {
		this.components = new ArrayList<IComponent>();
		this.joystick = new Joystick(Schematic.ID_JOYSTICK);
		this.gyro = new Gyro(Schematic.PORT_GYROSCOPE);

		this.components.add(new SwerveComponent(Schematic.USE_GYROSCOPE, Schematic.ROBOT_LENGTH, Schematic.ROBOT_WIDTH, gyro)); // Add Swerve Drive.
	}

	@Override
	public void autonomous() {
		while (isAutonomous() && isEnabled()) {
			for (IComponent c : components) {
				c.autonomousUpdate();
			}
		}
	}

	@Override
	public void test() {
		while (isTest() && isEnabled()) {
			for (IComponent c : components) {
				c.testUpdate();
			}
		}
	}

	@Override
	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			for (IComponent c : components) {
				c.operatorUpdate(joystick);
			}
		}
	}
}
