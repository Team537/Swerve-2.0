package org.robot;

import java.util.ArrayList;
import java.util.List;

import org.robot.components.IAutonomous;
import org.robot.components.IComponent;
import org.robot.swerve.SwerveComponent;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
	private List<IAutonomous> autonomousComponents;
	private List<IComponent> operatorComponents;
	private Joystick joystick1, joystick2;
	private Gyro gyro;

	@Override
	public void robotInit() {
		new SmartDashboard();
		
		this.autonomousComponents = new ArrayList<IAutonomous>();
		this.operatorComponents = new ArrayList<IComponent>();
		this.gyro = new Gyro(Schematic.PORT_GYRO);

		this.joystick1 = new Joystick(Schematic.JOYSTICK_MAIN); // Add the main joystick.
		this.joystick2 = new Joystick(Schematic.JOYSTICK_SECONDARY); // Add the secondary joystick.

		this.operatorComponents.add(new SwerveComponent(gyro)); // Add Swerve Drive component.
	}

	@Override
	public void autonomous() {
		long startTime = System.currentTimeMillis();

		while (isAutonomous() && isEnabled()) {
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
