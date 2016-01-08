package org.robot.swerve;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveModule {
	private PIDController anglePID;
	private CANTalon outputSpeed, outputAngle;
	private AnalogPotentiometer anglePOT;
	private String moduleName;
	private float angleOffset;

	public DriveModule(int speedPort, int anglePort, int potPort, PIDValue angleValuesPID, float angleOffset, String name) {
		this.outputSpeed = new CANTalon(speedPort);
		this.outputAngle = new CANTalon(anglePort);
		this.anglePOT = new AnalogPotentiometer(potPort, 360, 0);

		outputSpeed.enableControl();
		outputAngle.enableControl();

		this.anglePID = new PIDController(angleValuesPID.getP(), angleValuesPID.getI(), angleValuesPID.getD(), anglePOT, outputAngle);
		anglePID.disable();
		anglePID.reset();
		anglePID.setInputRange(angleValuesPID.getInputMin(), angleValuesPID.getInputMax());
		anglePID.setOutputRange(angleValuesPID.getOutputMin(), angleValuesPID.getOutputMax());
		anglePID.setAbsoluteTolerance(5.0);
		anglePID.setContinuous(true);
		anglePID.enable();

		this.moduleName = name;
		this.angleOffset = angleOffset;
	}

	public void drive(double speed) {
		outputSpeed.set(speed);
		// SmartDashboard.putNumber((moduleName + " Speed Drive Input"), speed);
	}

	public void steer(double angle) {
		angle += angleOffset;

		if (angle < 0) {
			angle += 360;
		} else if (angle >= 360) {
			angle -= 360;
		}

		anglePID.enable();
		anglePID.setSetpoint(angle);
		// SmartDashboard.putNumber((moduleName + " Angle Drive Input"), angle);
	}
	
	public void PIDSetAngle(float p, float i, float d) {
		anglePID.setPID(p, i, d);
	}

	public boolean PIDAtTargetAngle() {
		return anglePID.onTarget();
	}

	public void PIDResetAngle() {
		anglePID.reset();
	}

	public PIDController getAnglePID() {
		return anglePID;
	}

	public void dashboardPrint() {
		SmartDashboard.putNumber((moduleName + " POT Reading"), anglePOT.get());
		SmartDashboard.putNumber((moduleName + " POT PID"), anglePOT.pidGet());
		SmartDashboard.putNumber((moduleName + " Angle Error"), anglePID.getError());
	}
}
