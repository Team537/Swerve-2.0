package org.swerve;

import edu.wpi.first.wpilibj.CANTalon;

enum TypeTalon {
	SPEED, ANGLE, FOLLOWER
}

enum TypeReadings {
	POT, ANALOG_ENCODER, QUAD_ENCODER, NONE
}

public class TalonManager {
	private int port;
	private CANTalon talon;
	private TypeTalon talonType;
	private TypeReadings talonReadings;
	private double OutputCurrent, OutputVoltage, BusVoltage;
	private double TalonPosition, TalonVelocity, TalonError;

	/**
	 * Creates a new TalonManager, use this to easy make and update CanTalons.
	 * 
	 * @param port The port the CanTalon can be found on.
	 * @param type The general type of talon to be created, weather it be SPEED or ANGLE.
	 * @param readings What reading input type should be used, POT, ENCODER or NONE.
	 */
	TalonManager(int port, TypeTalon type, TypeReadings readings) {
		this.port = port;
		this.talon = new CANTalon(port);
		this.talonType = type;
		this.talonReadings = readings;

		switch (talonType) {
			case SPEED:
				talon.changeControlMode(CANTalon.ControlMode.PercentVbus);
				break;
			case ANGLE:
				talon.changeControlMode(CANTalon.ControlMode.Position);
				break;
			case FOLLOWER:
				talon.changeControlMode(CANTalon.ControlMode.Follower);
				break;
		}

		switch (talonReadings) {
			case POT:
				talon.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
				break;
			case ANALOG_ENCODER:
				talon.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogEncoder);
				break;
			case QUAD_ENCODER:
				talon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
				break;
			case NONE:
				break;
		}

		// Set a new target (0 speed || 0 angle).
		set(0);
	}

	public int getPort() {
		return port;
	}

	public double getOutputCurrent() {
		return OutputCurrent;
	}

	public double getOutputVoltage() {
		return OutputVoltage;
	}

	public double getBusVoltage() {
		return BusVoltage;
	}

	public double getTalonPosition() {
		return TalonPosition;
	}

	public double getTalonVelocity() {
		return TalonVelocity;
	}

	public double getTalonError() {
		return TalonError;
	}

	/**
	 * Sets a new PID value to this talon.
	 * 
	 * @param p P value of the new PID.
	 * @param i I value of the new PID.
	 * @param d D value of the new PID.
	 */
	void setPID(double p, double i, double d) {
		talon.setPID(p, i, d);
	}

	/**
	 * Sets the output value on the talon, depending on the mode.
	 *
	 * In SPEED, the value is between -1.0 and 1.0, with 0.0 as stopped.<br>
	 * In ANGLE, value is in encoder ticks or an analog value, depending on the sensor.<br>
	 * In FOLLOWER, the value is the integer device ID of the talon to duplicate.<br>
	 *
	 * @param value The setpoint value, as described above.
	 */
	void set(double value) {
		talon.set(value);
	}

	/**
	 * Gets the current status of the talon.
	 *
	 * In SPEED and FOLLOWER: returns current applied throttle.<br>
	 * In ANGLE: returns current sensor position.<br>
	 *
	 * @return The current sensor value of the talon.
	 */
	double get() {
		return talon.get();
	}

	void update() {
		this.OutputCurrent = talon.getOutputCurrent();
		this.OutputVoltage = talon.getOutputVoltage();
		this.BusVoltage = talon.getBusVoltage();

		// switch (talonType) {
		// case SPEED:
		// talon.getSpeed();
		// break;
		// case ANGLE:
		// talon.getPosition();
		// break;
		// case FOLLOWER:
		// break;
		// }

		switch (talonReadings) {
			case POT:
				this.TalonPosition = talon.getAnalogInPosition();
				this.TalonVelocity = talon.getAnalogInVelocity();
				this.TalonError = talon.getClosedLoopError();
				break;
			case ANALOG_ENCODER:
				this.TalonPosition = talon.getEncPosition();
				this.TalonVelocity = talon.getEncVelocity();
				this.TalonError = talon.getClosedLoopError();
				break;
			case QUAD_ENCODER:
				this.TalonPosition = talon.getEncPosition();
				this.TalonVelocity = talon.getEncVelocity();
				this.TalonError = talon.getClosedLoopError();
				break;
			case NONE:
				this.TalonPosition = -1;
				this.TalonVelocity = -1;
				this.TalonError = -1;
				break;
		}
	}

	/**
	 * Disables this talon and stops it from moving.
	 */
	void disable() {
		talon.disable();
	}
}
