package org.team537;

import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot {
	private Swerve swerve;

	@Override
	public void robotInit() {
		swerve = new Swerve();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopPeriodic() {
		swerve.drive();
		// swerve.tune(swerve.frontRightAngle);
		swerve.dashboard();
	}

	@Override
	public void testPeriodic() {
	}
}
