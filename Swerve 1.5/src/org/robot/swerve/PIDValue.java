package org.robot.swerve;

public class PIDValue {
	private float p, i, d, inputMin, inputMax, outputMin, outputMax;

	public PIDValue(float p, float i, float d, float inputMin, float inputMax, float outputMin, float outputMax) {
		this.p = p;
		this.i = i;
		this.d = d;
		this.inputMin = inputMin;
		this.inputMax = inputMax;
		this.outputMin = outputMin;
		this.outputMax = outputMax;
	}

	public float getP() {
		return p;
	}

	public float getI() {
		return i;
	}

	public float getD() {
		return d;
	}

	public float getInputMin() {
		return inputMin;
	}

	public float getInputMax() {
		return inputMax;
	}

	public float getOutputMin() {
		return outputMin;
	}

	public float getOutputMax() {
		return outputMax;
	}
}
