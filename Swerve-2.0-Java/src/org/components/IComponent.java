package org.components;

import edu.wpi.first.wpilibj.Joystick;

public abstract class IComponent {
	/**
	 * This is run during Operator Control Mode and is used for running updates the component.
	 * 
	 * @param joystick1 The main joystick that can be used to move the robot (the component should know the joystick ID).
	 * @param joystick2 The secondary joystick.
	 */
	public abstract void update(Joystick joystick1, Joystick joystick2);
}
