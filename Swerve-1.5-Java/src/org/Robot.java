package org;

import java.util.ArrayList;
import java.util.List;

import org.components.IAutonomous;
import org.components.IComponent;
import org.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;

public class Robot extends SampleRobot {
	private List<IComponent> operatorComponents;
	private List<IAutonomous> autonomousComponents;
	private Joystick joystick1, joystick2;
	private Gyro gyro;

	@Override
	public void robotInit() {
		this.operatorComponents = new ArrayList<IComponent>();
		this.autonomousComponents = new ArrayList<IAutonomous>();
		this.gyro = new Gyro(Schematic.PORT_GYRO);

		this.joystick1 = new Joystick(Schematic.JOYSTICK_MAIN); // Add the main joystick.
		this.joystick2 = new Joystick(Schematic.JOYSTICK_SECONDARY); // Add the secondary joystick.

		this.operatorComponents.add(new SwerveDrive(gyro)); // Add Swerve Drive.
	}

	@Override
	public void autonomous() {
		long startTime = System.currentTimeMillis();

		while (!isOperatorControl() && isEnabled()) {
			for (IAutonomous c : autonomousComponents) {
				c.update(startTime - System.currentTimeMillis());
			}
		}
	}

	@Override
	public void operatorControl() {
		while (isOperatorControl() && isEnabled()) {
			for (IComponent c : operatorComponents) {
				c.update(joystick1, joystick2);
			}
		}
	}
}