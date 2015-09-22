package com;

import java.awt.Color;
import java.awt.GraphicsEnvironment;

import com.swerve.auto.LinePlot;
import com.swerve.auto.PathPlanner;

public class Main {
	public static void main(String[] args) {
		// Create waypoint path, x y and heading. (X, Y, Rotate).
		double[][] waypoints = new double[][] { 
			{ 2, 2, 0 },

			{ 2, 7, 90 },

			{ 2, 12, 180 },

			{ 2, 17, 270 },

			{ 2, 22, 360 },

			{ 7, 22, 450 },

			{ 12, 22, 540 },

			{ 17, 22, 630 },

			{ 22, 22, 720 },

			{ 22, 17, 810 },

			{ 22, 12, 900 },

			{ 22, 7, 990 },

			{ 22, 2, 1080 } 
		};

		double totalTime = 10; // Seconds.
		double timeStep = 0.1; // Period of control loop on Rio, seconds.
		double robotTrackWidth = 2; // Distance between left and right wheels, feet.
		double robotTrackLength = 2;

		final PathPlanner path = new PathPlanner(waypoints);

		path.setPathAlpha(0.7);
		path.setPathBeta(0.3);
		path.setPathTolerance(0.0000001);
		path.calculate(totalTime, timeStep, robotTrackWidth, robotTrackLength);

		if (!GraphicsEnvironment.isHeadless()) {
			LinePlot fig2 = new LinePlot(path.getSmoothCenterVelocity(), null, Color.blue);
			fig2.yGridOn();
			fig2.xGridOn();
			fig2.setYLabel("Velocity (ft/sec)");
			fig2.setXLabel("Time (seconds)");
			fig2.setTitle("Velocity Profile for Left and Right Wheels \n Overall = Blue, LU = Cyan, RU = Magenta, LL = Green, LR = Red");
			fig2.addData(path.getSmoothLeftUpperVelocity(), Color.cyan);
			fig2.addData(path.getSmoothRightUpperVelocity(), Color.magenta);
			fig2.addData(path.getSmoothLeftLowerVelocity(), Color.green);
			fig2.addData(path.getSmoothRightLowerVelocity(), Color.red);

			LinePlot fig1 = new LinePlot(path.getOriginalPath(), Color.blue, Color.green);

			// Run a "simulation" of the paths with a "robot".
			for (int i = 0; i < path.getLeftUpperPath().length; i++) {
				fig1.frame.setLocation(700, 100); // make a static position for the window.
				fig1.frame.setSize(750, 750);
				fig1.yGridOn();
				fig1.xGridOn();
				fig1.setYLabel("Y (Feet)");
				fig1.setXLabel("X (Feet)");
				fig1.setTitle("Shows global position of robot path, along with left and right wheel trajectories");

				// Force graph to show 1/2 field dimensions of 24ft x 27 feet.
				fig1.setXTic(0, 27, 1);
				fig1.setYTic(0, 24, 1);

				// Draw our paths for each wheel.
				fig1.addData(path.getSmoothPath(), Color.orange);
				fig1.addData(path.getLeftUpperPath(), Color.magenta);
				fig1.addData(path.getLeftLowerPath(), Color.magenta);
				fig1.addData(path.getRightUpperPath(), Color.magenta);
				fig1.addData(path.getRightLowerPath(), Color.magenta);

				// Draw our "robot".
				double[][] topline = new double[][] { { path.getLeftUpperPath()[i][0], path.getLeftUpperPath()[i][1] }, { path.getRightUpperPath()[i][0], path.getRightUpperPath()[i][1] } };
				fig1.addData(topline, Color.black);

				double[][] leftline = new double[][] { { path.getLeftUpperPath()[i][0], path.getLeftUpperPath()[i][1] }, { path.getLeftLowerPath()[i][0], path.getLeftLowerPath()[i][1] } };
				fig1.addData(leftline, Color.black);

				double[][] rightline = new double[][] { { path.getRightUpperPath()[i][0], path.getRightUpperPath()[i][1] }, { path.getRightLowerPath()[i][0], path.getRightLowerPath()[i][1] } };
				fig1.addData(rightline, Color.black);

				double[][] bottomline = new double[][] { { path.getLeftLowerPath()[i][0], path.getLeftLowerPath()[i][1] }, { path.getRightLowerPath()[i][0], path.getRightLowerPath()[i][1] } };
				fig1.addData(bottomline, Color.black);

				double leftdist = Math.sqrt(Math.pow(path.getLeftUpperPath()[i][0] - path.getLeftLowerPath()[i][0], 2) + Math.pow(path.getLeftUpperPath()[i][1] - path.getLeftLowerPath()[i][1], 2));
				double topdist = Math.sqrt(Math.pow(path.getLeftUpperPath()[i][0] - path.getRightUpperPath()[i][0], 2) + Math.pow(path.getLeftUpperPath()[i][1] - path.getRightUpperPath()[i][1], 2));
				double bottomdist = Math.sqrt(Math.pow(path.getRightLowerPath()[i][0] - path.getLeftLowerPath()[i][0], 2) + Math.pow(path.getRightLowerPath()[i][1] - path.getLeftLowerPath()[i][1], 2));
				double rightdist = Math.sqrt(Math.pow(path.getRightLowerPath()[i][0] - path.getRightUpperPath()[i][0], 2) + Math.pow(path.getRightLowerPath()[i][1] - path.getRightUpperPath()[i][1], 2));

				double downdiagonaldist = Math.sqrt(Math.pow(path.getLeftUpperPath()[i][0] - path.getRightLowerPath()[i][0], 2) + Math.pow(path.getLeftUpperPath()[i][1] - path.getRightLowerPath()[i][1], 2));
				double updiagonaldist = Math.sqrt(Math.pow(path.getRightUpperPath()[i][0] - path.getLeftLowerPath()[i][0], 2) + Math.pow(path.getRightUpperPath()[i][1] - path.getLeftLowerPath()[i][1], 2));

				// Test to ensure that the paths are tranversable by a rigid body.
				if (Math.abs(leftdist + topdist + bottomdist + rightdist - robotTrackLength * 2 - robotTrackWidth * 2) > .001) {
					System.out.println("incorrect path(s)");
				}

				if (Math.abs(downdiagonaldist + updiagonaldist - 2 * Math.sqrt(8)) > .001) {
					System.out.println("incorrect path{s)");
				}

				// Nice little wait in a try / catch rather than throws.
				try {
					// Robot simulated driving along path at a constant speed. NOTE: real life robot experiences *will* change.
					Thread.sleep((long) (1000 * (totalTime / path.getSmoothCenterVelocity().length)));
				} catch (Exception e) {
					e.printStackTrace();
				}

				// Clear displayed data to update it.
				if (i < path.getLeftLowerPath().length - 1) {
					fig1.clearData();
				}
			}
		}
	}
}
