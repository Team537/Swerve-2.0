package org.swerve;

public class PIDDistance {
	private float p, i, d;

	public PIDDistance(float p, float i, float d) {
		this.p = p;
		this.i = i;
		this.d = d;
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
}
