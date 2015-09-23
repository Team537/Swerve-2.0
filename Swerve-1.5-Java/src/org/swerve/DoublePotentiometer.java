package org.swerve;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class DoublePotentiometer extends AnalogPotentiometer {
	private final int OVERSAMPLE = 2;
	private float offset, min, max, range;
	private String name;
	private double lastReading;
	private int counter;
	private double data[];

	public DoublePotentiometer(int channel, double fullRange, float offset, float min, float max, String name) {
		super(channel, fullRange, offset);

		this.min = min;
		this.max = max;
		this.range = max - min;
		this.offset = offset;
		this.name = name;
		this.lastReading = 0;
		this.counter = 0;
		this.data = new double[OVERSAMPLE];

		for (int i = 0; i < OVERSAMPLE; i++) {
			data[i] = 0;
		}
	}

	@Override
	public double pidGet() {
		double currentReading = super.get();
		double delta = currentReading - lastReading;
		double sum = 0;

		if (delta > 300) {
			// Make sure that when looping past reset point it doesn't go backwards.
			// return lastval;
			currentReading = lastReading;
		}

		lastReading = currentReading;
		currentReading += offset;

		if (currentReading > max) {
			currentReading -= range;
		} else if (currentReading < min) {
			currentReading += range;
		}

		// Run oversample code.
		for (int i = OVERSAMPLE - 1; i > 0; i--) {
			data[i - 1] = data[i];
		}

		data[OVERSAMPLE - 1] = currentReading;
		counter++;

		if (counter >= OVERSAMPLE) {
			for (int i = 0; i < OVERSAMPLE; i++) {
				sum += data[i];
			}

			return sum / OVERSAMPLE;
		}

		return currentReading;
	}

	public double getAverage() {
		double sum = 0;

		for (int i = 0; i < OVERSAMPLE; i++) {
			sum += data[i];
		}

		return sum / OVERSAMPLE;
	}

	public String getName() {
		return name;
	}
}
