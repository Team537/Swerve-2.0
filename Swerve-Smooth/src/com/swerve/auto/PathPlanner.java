package com.swerve.auto;

/**
 * This Class provides many useful algorithms for Robot Path Planning. It uses optimization techniques and knowledge of Robot Motion in order to calculate smooth path trajectories, if given only discrete waypoints. The Benefit of these optimization algorithms are very efficient path planning that can be used to Navigate in Real-time.
 *
 * This Class uses a method of Gradient Decent, and other optimization techniques to produce smooth Velocity profiles for every wheel of a 4 wheeled swerve drive.
 *
 * This Class does not attempt to calculate quintic or cubic splines for best fitting a curve. It is for this reason, the algorithm can be ran on embedded devices with very quick computation times.
 *
 * The output of this function are independent velocity profiles for the each wheel of a 4 wheeled swerve drivetrain. The velocity profiles start and end with 0 velocity and maintain smooth transitions throughout the path.
 */
public class PathPlanner {
	/** The waypoint only path */
	private double[][] originalPath;

	/** The origPath with injected points to allow for smooth transitions */
	private double[][] smoothPath;

	/** Swerve paths generated based off smoothPath */
	private double[][] leftUpperPath;
	private double[][] rightUpperPath;
	private double[][] leftLowerPath;
	private double[][] rightLowerPath;

	private double[][] smoothCenterVelocity;
	private double[][] smoothLeftUpperVelocity;
	private double[][] smoothRightUpperVelocity;
	private double[][] smoothLeftLowerVelocity;
	private double[][] smoothRightLowerVelocity;

	// Tuning parameters for tuning the path generated.
	double pathAlpha;
	double pathBeta;
	double pathTolerance;

	// Tuning paramters for the velocities generated.
	double velocityAlpha;
	double velocityBeta;
	double velocityTolerance;

	/**
	 * Constructor, takes a Path of Way Points defined as a double array of column vectors representing the global cartesian points of the path in {x, y, heading} coordinates. The waypoint are traveled from one point to the next in sequence.
	 *
	 * For example: here is a properly formated waypoint array.
	 *
	 * double[][] waypointPath = new double[][]{ {1, 1, 0}, {5, 1, 45}, {9, 12, 90}, {12, 9, 135}, {15,6, 180}, {15, 4, 225} }; This path goes from {1,1,0} -> {5,1,45} -> {9,12,90} -> {12,9, 135} -> {15,6,180} -> {15,4,225} The units of these coordinates are position units assumed by the user (i.e inch, foot, meters)
	 * 
	 * @param path
	 */
	public PathPlanner(double[][] path) {
		this.originalPath = doubleArrayCopy(path);

		// Default values DO NOT MODIFY;
		this.pathAlpha = 0.7;
		this.pathBeta = 0.3;
		this.pathTolerance = 0.0000001;

		this.velocityAlpha = 0.1;
		this.velocityBeta = 0.3;
		this.velocityTolerance = 0.0000001;
	}

	/**
	 * Performs a deep copy of a 2 Dimensional Array looping thorough each element in the 2D array.
	 *
	 * BigO: Order N x M.
	 * 
	 * @param array
	 * @return
	 */
	public static double[][] doubleArrayCopy(double[][] array) {
		// Size first dimension of array.
		double[][] temp = new double[array.length][array[0].length];

		for (int i = 0; i < array.length; i++) {
			// Resize second dimension of array.
			temp[i] = new double[array[i].length];

			// Copy Contents.
			System.arraycopy(array[i], 0, temp[i], 0, array[i].length);
		}

		return temp;
	}

	/**
	 * Method upsamples the Path by linear injection. The result providing more waypoints along the path.
	 *
	 * BigO: Order N * injection#
	 *
	 * @param original
	 * @param numToInject
	 * @return
	 */
	public double[][] inject(double[][] original, int numToInject) {
		// Create extended 2 Dimensional array to hold additional points.
		double[][] morePoints = new double[original.length + ((numToInject) * (original.length - 1))][3];

		int index = 0;

		// Loop through original array.
		for (int i = 0; i < original.length - 1; i++) {
			// Copy first.
			morePoints[index][0] = original[i][0];
			morePoints[index][1] = original[i][1];
			morePoints[index][2] = original[i][2];
			index++;

			for (int j = 1; j < numToInject + 1; j++) {
				// Calculate intermediate x points between j and j+1 original points.
				morePoints[index][0] = j * ((original[i + 1][0] - original[i][0]) / (numToInject + 1)) + original[i][0];

				// Calculate intermediate y points between j and j+1 original points.
				morePoints[index][1] = j * ((original[i + 1][1] - original[i][1]) / (numToInject + 1)) + original[i][1];

				// Calculate intermediate heading points between j and j+1 original points.
				morePoints[index][2] = j * ((original[i + 1][2] - original[i][2]) / (numToInject + 1)) + original[i][2];

				index++;
			}
		}

		// Copy last points.
		morePoints[index][0] = original[original.length - 1][0];
		morePoints[index][1] = original[original.length - 1][1];
		morePoints[index][2] = original[original.length - 1][2];

		return morePoints;
	}

	/**
	 * Optimization algorithm, which optimizes the data points in path to create a smooth trajectory. This optimization uses gradient descent. While unlikely, it is possible for this algorithm to never converge. If this happens, try increasing the tolerance level.
	 *
	 * BigO: N^x, where X is the number of of times the while loop iterates before tolerance is met.
	 *
	 * @param path
	 * @param weightData
	 * @param weightSmooth
	 * @param tolerance
	 * @return
	 */
	public double[][] smoother(double[][] path, double weightData, double weightSmooth, double tolerance) {
		// Copy array
		double[][] newPath = doubleArrayCopy(path);
		double change = tolerance;

		while (change >= tolerance) {
			change = 0.0;

			for (int i = 1; i < path.length - 1; i++) {
				for (int j = 0; j < path[i].length; j++) {
					double aux = newPath[i][j];
					newPath[i][j] += weightData * (path[i][j] - newPath[i][j]) + weightSmooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
					change += Math.abs(aux - newPath[i][j]);
				}
			}
		}

		return newPath;
	}

	/**
	 * Returns Velocity as a double array. The First Column vector is time, based on the time step, the second vector is the velocity magnitude.
	 *
	 * BigO: order N
	 * 
	 * @param smoothPath
	 * @param timeStep
	 * @return
	 */
	double[][] velocity(double[][] smoothPath, double timeStep) {
		double[] dxdt = new double[smoothPath.length];
		double[] dydt = new double[smoothPath.length];
		double[][] velocity = new double[smoothPath.length][3];

		// Set first instance to zero.
		dxdt[0] = 0;
		dydt[0] = 0;
		velocity[0][0] = 0;
		velocity[0][1] = 0;
		velocity[0][2] = 0;

		for (int i = 1; i < smoothPath.length; i++) {
			dxdt[i] = (smoothPath[i][0] - smoothPath[i - 1][0]) / timeStep;
			dydt[i] = (smoothPath[i][1] - smoothPath[i - 1][1]) / timeStep;

			// Create time vector.
			velocity[i][0] = velocity[i - 1][0] + timeStep;

			// Calculate velocity.
			velocity[i][1] = Math.sqrt(Math.pow(dxdt[i], 2) + Math.pow(dydt[i], 2));

			// Create heading vector.
			velocity[i][2] = smoothPath[i][2];
		}

		return velocity;
	}

	double[][] velocity(double[][] wheelPath, double[][] smoothPath, double timeStep, double robotTrackWidth, double robotTrackLength, String id) {
		double[] dxdt = new double[wheelPath.length];
		double[] dydt = new double[wheelPath.length];
		double[][] velocity = new double[wheelPath.length][3];

		// Set first instance to zero.
		dxdt[0] = 0;
		dydt[0] = 0;
		velocity[0][0] = 0;
		velocity[0][1] = 0;
		velocity[0][2] = smoothPath[0][2];

		for (int i = 1; i < smoothPath.length; i++) {
			smoothPath[i][2] = Math.toRadians(smoothPath[i][2]);
			dxdt[i] = (wheelPath[i][0] - wheelPath[i - 1][0]) / timeStep;
			dydt[i] = (wheelPath[i][1] - wheelPath[i - 1][1]) / timeStep;

			while (smoothPath[i][2] >= Math.PI * 2) {
				smoothPath[i][2] = smoothPath[i][2] - Math.PI * 2;
			}

			if (smoothPath[i][2] > Math.PI) {
				smoothPath[i][2] = smoothPath[i][2] - Math.PI * 2;
			}

			// Create time vector.
			velocity[i][0] = velocity[i - 1][0] + timeStep;

			// Calculate velocity[1] and heading[2].
			switch (id) {
				case "LU":
					velocity[i][1] = Math.sqrt(Math.pow(dxdt[i] + smoothPath[i][2] * (-robotTrackWidth / 2 + robotTrackLength / 2), 2) + Math.pow(dydt[i] - smoothPath[i][2] * (-robotTrackWidth / 2 + robotTrackLength / 2), 2));
					velocity[i][2] = (180 / Math.PI) * Math.atan2((dxdt[i] + smoothPath[i][2] * (-robotTrackWidth / 2 + robotTrackLength / 2)), (dydt[i] - smoothPath[i][2] * (-robotTrackWidth / 2 + robotTrackLength / 2)));
					break;
				case "LL":
					velocity[i][1] = Math.sqrt(Math.pow(dxdt[i] + smoothPath[i][2] * (-robotTrackWidth / 2 + robotTrackLength / 2), 2) + Math.pow(dydt[i] - smoothPath[i][2] * (-robotTrackWidth / 2 + robotTrackLength / 2), 2));
					velocity[i][2] = (180 / Math.PI) * Math.atan2((dxdt[i] + smoothPath[i][2] * (-robotTrackWidth / 2 + robotTrackLength / 2)), (dydt[i] - smoothPath[i][2] * (-robotTrackWidth / 2 + robotTrackLength / 2)));
					break;
				case "RU":
					velocity[i][1] = Math.sqrt(Math.pow(dxdt[i] + smoothPath[i][2] * (robotTrackWidth / 2 + robotTrackLength / 2), 2) + Math.pow(dydt[i] - smoothPath[i][2] * (robotTrackWidth / 2 + robotTrackLength / 2), 2));
					velocity[i][2] = (180 / Math.PI) * Math.atan2((dxdt[i] + smoothPath[i][2] * (robotTrackWidth / 2 + robotTrackLength / 2)), (dydt[i] - smoothPath[i][2] * (robotTrackWidth / 2 + robotTrackLength / 2)));
					break;
				case "RL":
					velocity[i][1] = Math.sqrt(Math.pow(dxdt[i] + smoothPath[i][2] * (robotTrackWidth / 2 - robotTrackLength / 2), 2) + Math.pow(dydt[i] - smoothPath[i][2] * (robotTrackWidth / 2 - robotTrackLength / 2), 2));
					velocity[i][2] = (180 / Math.PI) * Math.atan2((dxdt[i] + smoothPath[i][2] * (robotTrackWidth / 2 - robotTrackLength / 2)), (dydt[i] - smoothPath[i][2] * (robotTrackWidth / 2 - robotTrackLength / 2)));
					break;
			}
		}

		return velocity;
	}

	/**
	 * Optimize velocity by minimizing the error distance at the end of travel when this function converges, the fixed velocity vector will be smooth, start and end with 0 velocity, and travel the same final distance as the original un-smoothed velocity profile
	 *
	 * This Algorithm may never converge. If this happens, reduce tolerance.
	 *
	 * @param smoothVelocity
	 * @param originalVelocity
	 * @param tolerance
	 * @return
	 */
	double[][] velocityFix(double[][] smoothVelocity, double[][] originalVelocity, double tolerance) {
		/**
		 * Pseudo code:<br>
		 * 1. Find Error Between Original Velocity and Smooth Velocity.<br>
		 * 2. Keep increasing the velocity between the first and last node of the smooth Velocity by a small amount.<br>
		 * 3. Recalculate the difference, stop if threshold is met or repeat step 2 until the final threshold is met.<br>
		 * 3. Return the updated smoothVelocity.<br>
		 */

		// Calculate error difference.
		double[] difference = errorSum(originalVelocity, smoothVelocity);

		// Copy smooth velocity into new Vector.
		double[][] fixVelocity = new double[smoothVelocity.length][2];

		for (int i = 0; i < smoothVelocity.length; i++) {
			fixVelocity[i][0] = smoothVelocity[i][0];
			fixVelocity[i][1] = smoothVelocity[i][1];
		}

		// Optimize velocity by minimizing the error distance at the end of travel when this converges, the fixed velocity vector will be smooth, start
		// and end with 0 velocity, and travel the same final distance as the original un-smoothed velocity profile.
		double increase;

		while (Math.abs(difference[difference.length - 1]) > tolerance) {
			increase = difference[difference.length - 1] / 1 / 50;

			for (int i = 1; i < fixVelocity.length - 1; i++) {
				fixVelocity[i][1] = fixVelocity[i][1] - increase;
			}

			difference = errorSum(originalVelocity, fixVelocity);
		}

		return fixVelocity;
	}

	/**
	 * This method calculates the integral of the Smooth Velocity term and compares it to the Integral of the original velocity term. In essence we are comparing the total distance by the original velocity path and the smooth velocity path to ensure that as we modify the smooth Velocity it still covers the same distance as was intended by the original velocity path.
	 *
	 * BigO: Order N
	 * 
	 * @param originalVelocity
	 * @param smoothVelocity
	 * @return
	 */
	private double[] errorSum(double[][] originalVelocity, double[][] smoothVelocity) {
		// Copy vectors.
		double[] tempOrigDist = new double[originalVelocity.length];
		double[] tempSmoothDist = new double[smoothVelocity.length];
		double[] difference = new double[smoothVelocity.length];

		double timeStep = originalVelocity[1][0] - originalVelocity[0][0];

		// Copy first elements.
		tempOrigDist[0] = originalVelocity[0][1];
		tempSmoothDist[0] = smoothVelocity[0][1];

		// Calculate difference.
		for (int i = 1; i < originalVelocity.length; i++) {
			tempOrigDist[i] = originalVelocity[i][1] * timeStep + tempOrigDist[i - 1];
			tempSmoothDist[i] = smoothVelocity[i][1] * timeStep + tempSmoothDist[i - 1];
			difference[i] = tempSmoothDist[i] - tempOrigDist[i];
		}

		return difference;
	}

	/**
	 * This method calculates the optimal parameters for determining what amount of nodes to inject into the path to meet the time restraint. This approach uses an iterative process to inject and smooth, yielding more desirable results for the final smooth path.
	 *
	 * Big O: Constant Time
	 *
	 * @param numNodeOnlyPoints
	 * @param maxTimeToComplete
	 * @param timeStep
	 */
	public int[] injectionCounter2Steps(double numNodeOnlyPoints, double maxTimeToComplete, double timeStep) {
		int first = 0;
		int second = 0;
		int third = 0;

		double oldPointsTotal = 0;
		int[] ret;
		double totalPoints = maxTimeToComplete / timeStep;

		if (totalPoints < 100) {
			double pointsFirst;
			double pointsTotal;

			for (int i = 4; i <= 6; i++)
				for (int j = 1; j <= 8; j++) {
					pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
					pointsTotal = (j * (pointsFirst - 1) + pointsFirst);

					if (pointsTotal <= totalPoints && pointsTotal > oldPointsTotal) {
						first = i;
						second = j;
						oldPointsTotal = pointsTotal;
					}
				}

			ret = new int[] { first, second, third };
		} else {
			double pointsFirst;
			double pointsSecond;
			double pointsTotal;

			for (int i = 1; i <= 5; i++) {
				for (int j = 1; j <= 8; j++) {
					for (int k = 1; k < 8; k++) {
						pointsFirst = i * (numNodeOnlyPoints - 1) + numNodeOnlyPoints;
						pointsSecond = (j * (pointsFirst - 1) + pointsFirst);
						pointsTotal = (k * (pointsSecond - 1) + pointsSecond);

						if (pointsTotal <= totalPoints) {
							first = i;
							second = j;
							third = k;
						}
					}
				}
			}

			ret = new int[] { first, second, third };
		}

		return ret;
	}

	/**
	 * Calculates the left and right wheel paths based on robot track width.
	 *
	 * Big O: 2N
	 *
	 * @param smoothPath - center smooth path of robot
	 * @param robotTrackWidth - width between left and right wheels of robot of skid steer chassis.
	 */
	public void paths(double[][] smoothPath, double robotTrackWidth, double robotTrackLength) {
		double[][] leftUpperPath = new double[smoothPath.length][2];
		double[][] leftLowerPath = new double[smoothPath.length][2];
		double[][] rightUpperPath = new double[smoothPath.length][2];
		double[][] rightLowerPath = new double[smoothPath.length][2];

		double[] gradient = new double[smoothPath.length];

		for (int i = 0; i < smoothPath.length - 1; i++) {
			gradient[i] = smoothPath[i][2];
			gradient[i] = Math.toRadians(gradient[i]);
		}

		for (int i = 0; i < gradient.length; i++) {
			leftLowerPath[i][0] = -(robotTrackWidth / 2 * Math.cos(gradient[i] - Math.PI / 2)) - (robotTrackLength / 2 * Math.sin(gradient[i] + Math.PI / 2)) + smoothPath[i][0];
			leftLowerPath[i][1] = -(robotTrackWidth / 2 * Math.sin(gradient[i] + Math.PI / 2)) - (robotTrackLength / 2 * Math.cos(gradient[i] + Math.PI / 2)) + smoothPath[i][1];

			rightLowerPath[i][0] = (robotTrackWidth / 2 * Math.cos(gradient[i] - Math.PI / 2)) + (robotTrackLength / 2 * Math.sin(gradient[i] + Math.PI / 2)) + smoothPath[i][0];
			rightLowerPath[i][1] = (robotTrackWidth / 2 * Math.sin(gradient[i] + Math.PI / 2)) + (robotTrackLength / 2 * Math.cos(gradient[i] + Math.PI / 2)) + smoothPath[i][1];

			leftUpperPath[i][0] = -(robotTrackWidth / 2 * Math.cos(gradient[i] + Math.PI / 2)) - (robotTrackLength / 2 * Math.sin(gradient[i] + Math.PI / 2)) + smoothPath[i][0];
			leftUpperPath[i][1] = -(robotTrackWidth / 2 * Math.sin(gradient[i] - Math.PI / 2)) - (robotTrackLength / 2 * Math.cos(gradient[i] + Math.PI / 2)) + smoothPath[i][1];

			rightUpperPath[i][0] = (robotTrackWidth / 2 * Math.cos(gradient[i] + Math.PI / 2)) + (robotTrackLength / 2 * Math.sin(gradient[i] + Math.PI / 2)) + smoothPath[i][0];
			rightUpperPath[i][1] = (robotTrackWidth / 2 * Math.sin(gradient[i] - Math.PI / 2)) + (robotTrackLength / 2 * Math.cos(gradient[i] + Math.PI / 2)) + smoothPath[i][1];

			// Convert gradient back to degrees.
			gradient[i] = Math.toDegrees(gradient[i]);
		}

		// Copy paths and heading into class.
		this.leftUpperPath = leftUpperPath;
		this.leftLowerPath = leftLowerPath;
		this.rightUpperPath = rightLowerPath;
		this.rightLowerPath = rightUpperPath;

		for (int i = 0; i < gradient.length - 1; i++) {
			this.smoothPath[i][2] = gradient[i];
		}
	}

	/**
	 * This code will calculate a smooth path based on the program parameters. If the user doesn't set any parameters, the will use the defaults optimized for most cases. The results will be saved into the corresponding class members. The user can then access .smoothPath, .leftPath, .rightPath, .smoothCenterVelocity, .smoothRightVelocity, .smoothLeftVelocity as needed.
	 *
	 * After calling this method, the user only needs to pass .smoothRightVelocity[1], .smoothLeftVelocity[1] to the corresponding speed controllers on the Robot, and step through each setPoint.
	 *
	 * @param totalTime - time the user wishes to complete the path in seconds. (this is the maximum amount of time the robot is allowed to take to traverse the path.)
	 * @param timeStep - the frequency at which the robot controller is running on the robot.
	 * @param robotTrackWidth - distance between left and right side wheels of a skid steer chassis. Known as the track width.
	 */
	public void calculate(double totalTime, double timeStep, double robotTrackWidth, double robotTrackLength) {
		/**
		 * Pseudo code.
		 *
		 * 1. Reduce input waypoints to only essential (direction changing) node points.<br>
		 * 2. Calculate how many total datapoints we need to satisfy the controller for "playback".<br>
		 * 3. Simultaneously inject and smooth the path until we end up with a smooth path with required number of datapoints, and which follows the waypoint path.<br>
		 * 4. Calculate left and right wheel paths by calculating parallel points at each datapoint.<br>
		 */

		// Figure out how many nodes to inject.
		int[] inject = injectionCounter2Steps(originalPath.length, totalTime, timeStep);

		// Iteratively inject and smooth the path.
		for (int i = 0; i < inject.length; i++) {
			if (i == 0) {
				smoothPath = inject(originalPath, inject[0]);
				smoothPath = smoother(smoothPath, pathAlpha, pathBeta, pathTolerance);
			} else {
				smoothPath = inject(smoothPath, inject[i]);
				smoothPath = smoother(smoothPath, pathAlpha, pathBeta, pathTolerance);
			}
		}

		// Calculate UL, UP, LL, LR from smoothPath.
		paths(smoothPath, robotTrackWidth, robotTrackLength);

		double[][] origCenterVelocity = velocity(smoothPath, timeStep);
		double[][] origLeftUpperVelocity = velocity(leftUpperPath, smoothPath, timeStep, robotTrackWidth, robotTrackLength, "LU");
		double[][] origLeftLowerVelocity = velocity(leftLowerPath, smoothPath, timeStep, robotTrackWidth, robotTrackLength, "LL");
		double[][] origRightUpperVelocity = velocity(rightUpperPath, smoothPath, timeStep, robotTrackWidth, robotTrackLength, "RU");
		double[][] origRightLowerVelocity = velocity(rightLowerPath, smoothPath, timeStep, robotTrackWidth, robotTrackLength, "RL");

		// Copy smooth velocities into fix Velocities.
		smoothCenterVelocity = doubleArrayCopy(origCenterVelocity);
		smoothLeftUpperVelocity = doubleArrayCopy(origLeftUpperVelocity);
		smoothLeftLowerVelocity = doubleArrayCopy(origLeftLowerVelocity);
		smoothRightUpperVelocity = doubleArrayCopy(origRightUpperVelocity);
		smoothRightLowerVelocity = doubleArrayCopy(origRightLowerVelocity);

		// Set final velocity to zero.
		smoothCenterVelocity[smoothCenterVelocity.length - 1][1] = 0.0;
		smoothLeftLowerVelocity[smoothLeftLowerVelocity.length - 1][1] = 0.0;
		smoothRightUpperVelocity[smoothRightUpperVelocity.length - 1][1] = 0.0;
		smoothRightLowerVelocity[smoothRightLowerVelocity.length - 1][1] = 0.0;

		// Smooth velocity with zero final V.
		smoothCenterVelocity = smoother(smoothCenterVelocity, velocityAlpha, velocityBeta, velocityTolerance);
		smoothLeftUpperVelocity = smoother(smoothLeftUpperVelocity, velocityAlpha, velocityBeta, velocityTolerance);
		smoothLeftLowerVelocity = smoother(smoothLeftLowerVelocity, velocityAlpha, velocityBeta, velocityTolerance);
		smoothRightUpperVelocity = smoother(smoothRightUpperVelocity, velocityAlpha, velocityBeta, velocityTolerance);
		smoothRightLowerVelocity = smoother(smoothRightUpperVelocity, velocityAlpha, velocityBeta, velocityTolerance);

		// Fix velocity distance error.
		smoothCenterVelocity = velocityFix(smoothCenterVelocity, origCenterVelocity, 0.0000001);
		smoothLeftUpperVelocity = velocityFix(smoothLeftUpperVelocity, origLeftUpperVelocity, 0.0000001);
		smoothLeftLowerVelocity = velocityFix(smoothLeftLowerVelocity, origLeftLowerVelocity, 0.0000001);
		smoothRightUpperVelocity = velocityFix(smoothRightUpperVelocity, origRightUpperVelocity, 0.0000001);
		smoothRightLowerVelocity = velocityFix(smoothRightLowerVelocity, origRightLowerVelocity, 0.0000001);
	}

	public double[][] getOriginalPath() {
		return originalPath;
	}

	public double[][] getSmoothPath() {
		return smoothPath;
	}

	public double[][] getLeftUpperPath() {
		return leftUpperPath;
	}

	public double[][] getRightUpperPath() {
		return rightUpperPath;
	}

	public double[][] getLeftLowerPath() {
		return leftLowerPath;
	}

	public double[][] getRightLowerPath() {
		return rightLowerPath;
	}

	public double[][] getSmoothCenterVelocity() {
		return smoothCenterVelocity;
	}

	public double[][] getSmoothLeftUpperVelocity() {
		return smoothLeftUpperVelocity;
	}

	public double[][] getSmoothRightLowerVelocity() {
		return smoothRightLowerVelocity;
	}

	public double[][] getSmoothLeftLowerVelocity() {
		return smoothLeftLowerVelocity;
	}

	public double[][] getSmoothRightUpperVelocity() {
		return smoothRightUpperVelocity;
	}

	public void setPathAlpha(double alpha) {
		pathAlpha = alpha;
	}

	public void setPathBeta(double beta) {
		pathBeta = beta;
	}

	public void setPathTolerance(double tolerance) {
		pathTolerance = tolerance;
	}
}