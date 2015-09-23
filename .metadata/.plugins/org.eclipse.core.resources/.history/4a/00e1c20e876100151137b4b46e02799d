package org.components;

import edu.wpi.first.wpilibj.Joystick;

public abstract class IComponent {
	/**
	 * This is run during Autonomous Mode and is used for running updates the component.
	 */
	public abstract void autonomousUpdate();
	
	/**
	 * This is run during Test Mode and is used for running updates the component.
	 */
	public abstract void testUpdate();
	
	/**
	 * This is run during Operator Control Mode and is used for running updates the component.
	 * 
	 * @param joystick The joysticks that can be used to move the robot.
	 */
	public abstract void operatorUpdate(Joystick joystick);

	/**
	 * This is called when the robot is being disabled. Can be used to cleanup.
	 */
	public abstract void dispose();
}
