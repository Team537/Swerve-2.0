package org.components;

public abstract class IAutonomous {
	/**
	 * Runs a update on this Autonomous component,
	 * 
	 * @param time The time sense the start of this autonomous period (measured in milliseconds)
	 */
	public abstract void update(long time);

	/**
	 * Called when the autonomous period has ended. Use this too wrap up this components tasks.
	 */
	public abstract void dispose();
}
