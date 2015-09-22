package org.swerve;

public class PIDDriveValue {
	private float p, i, d, f, rate;

	public PIDDriveValue(float p, float i, float d, float f, float rate) {
		this.p = p;
		this.i = i;
		this.d = d;
		this.f = f;
		this.rate = rate;
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

	public float getF() {
		return f;
	}

	public float getRate() {
		return rate;
	}
}
