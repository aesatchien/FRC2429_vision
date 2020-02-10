package org.usfirst.frc.team2429.spartan2017.subsystems;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
//import edu.wpi.first.wpilibj.vision.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;

import org.opencv.core.Mat;
import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;
import org.usfirst.frc.team2429.spartan2017.imaging.ImageCaptureAndProcess;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

/**
 *
 */
public class ImageProcessor extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private static NetworkTable gearTable = NetworkTable.getTable("GearCam");

	// Use these variable to store the data from an image snapshot
	// Then use them to move twice since the target usually goes out of the FOV
	// during a large move - 2/3/2017 CJH
	public static double autonomousGearDistance;
	public static double autonomousGearStrafe;
	public static double autonomousGearRotation;
	
	// Store the shooter variables so we can call them from a command
	public static double autonomousShooterRotation;
	public static double autonomousShooterDistance;
	
	// this is the only one we actually need to store when we rotate to the
	// target using the rPi
	public static double strafeCorrection;

	// All these are here in case we do image processing on-robot
	// We aren't since the raspberry pi seems to be working really well
	// But it's all here as a backup - 2/3/2017 CJH
	public static double[] gearTargetAreas;
	public static double[] gearTargetX;
	public static boolean targetsAcquired;
	public static boolean offBoardImaging;
	// public static double[] gearTargetAreaAveragesLeftArray;
	// public static double[] gearTargetAreaAveragesRightArray;
	// public static double gearTargetAreaAveragesLeft;
	// public static double gearTargetAreaAveragesRight;
	public static double[] gearTargetY;
	public static double[] gearTargetHeights;
	// public static double[] gearTargetWidths;
	public static double gearTargetCount;
	public static double gearTargetDistance;
	public static double gearTargetStrafe;
	public static double gearTargetRotation;
	// public static double[] gearTargetRotationArray;
	// int targetAverages;
	// int averageCounter;

	public static double shooterTargetRotation;
	public static double shooterTargetDistance;

	public static boolean isShootercamConnected;
	public static boolean isGearcamConnected;
	
	public void initDefaultCommand() {

		// Set the default command for a subsystem here.
		// setDefaultCommand(new ImageCaptureAndProcess());

		// Set this if the raspberrypi is doing all the imaging work for us.
		offBoardImaging = true;
		targetsAcquired = false;
		gearTargetAreas = new double[2];
		gearTargetX = new double[2];

		gearTargetY = new double[2];
		gearTargetHeights = new double[2];
		// gearTargetWidths= new double[2];
		// trying to get around the problem of averaging the frames - hard to
		// call a command iteratively
		// targetAverages=5;
		// averageCounter=0;
		// gearTargetAreaAveragesLeftArray = new double[targetAverages];
		// gearTargetAreaAveragesRightArray = new double[targetAverages];
		// gearTargetRotationArray = new double[10];
	}

	/**
	 * This is the important one. Calculate the strafe correction necessary to
	 * correct turning an angle that takes the target outside the camera FOV
	 * 
	 */
	public static void setStrafeCorrection() {
		// Have to keep the signs right - Sin(x) is odd, so it carries the sign
		// with it
		// if i turn clockwise, i need to go backwards
		strafeCorrection = -(gearTargetDistance * Math.sin(gearTargetRotation * Math.PI / 180.0));
		SmartDashboard.putNumber("Strafe Correction", strafeCorrection);
	}

	public static double getStrafeCorrection() {
		return strafeCorrection;
	}

	public static double getAutonomousGearDistance() {
		return autonomousGearDistance;
	}

	public static void setAutonomousGearDistance() {
		ImageProcessor.autonomousGearDistance = gearTable.getNumber("distanceByHeight", -1.0);
	}

	public static double getAutonomousGearStrafe() {
		return autonomousGearStrafe;
	}

	public static void setAutonomousGearStrafe() {
		ImageProcessor.autonomousGearStrafe = gearTable.getNumber("strafe", 0.0);
	}

	public static double getAutonomousGearRotation() {
		return autonomousGearRotation;
	}

	public static void setAutonomousGearRotation() {
		ImageProcessor.autonomousGearRotation = gearTable.getNumber("rotation", 0.0);
	}

	/**
	 * There are multiple ways to calculate the distance to the gear target.
	 * Most of the ones on the web are wrong Most use the small approx for
	 * tangent and it's wrong and gets worse the farther you are away from the
	 * target Mine is right 01/26/2017 CJH
	 */

	@Deprecated
	// Only use this if we are doing on-board imaging. We're not, as of now, so
	// don't use this.
	public double getGearTargetDistance() {
		// Use trig to return distance to target based on 8.25 inch separation
		double distanceToTarget = 0;
		if (Robot.imageProcessor.targetsAcquired) {
			boolean debugBoolean = false;
			// gear targets are 8.25 inch separation
			double actualTargetWidth = 8.25;
			// total (not half) angular FOV of camera in radians
			double fovRadians = (Math.PI / 180.0) * (RobotMap.gearCamFov);
			// Based on sorting with farthest left at index [0] this gives pixel
			// dist
			// in our resolution agnostic -1 to 1 coordinate system; same as
			// (pixels width)/horiz_resolution
			double imageTargetWidth = (gearTargetX[1] - gearTargetX[0]) / 2.0;
			if (debugBoolean) {
				// The standard one that you get from Chief Delphi, etc, it
				// underestimates when you are far away
				distanceToTarget = (1.0 / imageTargetWidth) * (actualTargetWidth / 2.0) / (Math.tan(fovRadians / 2.0));
				System.out.print("Distance: Standard: " + String.format("%.2f", distanceToTarget));
				// Derived from a circle, will slightly overestimate. Note the
				// angle is not divided by 2 here
				distanceToTarget = (1.0 / imageTargetWidth) * actualTargetWidth / fovRadians;
				System.out.print(" | Arc approx : " + String.format("%.2f", distanceToTarget));
				// True formula, might as well use it
				distanceToTarget = actualTargetWidth / (2.0 * Math.tan(imageTargetWidth * fovRadians / 2.0));
				System.out.println(" | CJH  eqn : " + String.format("%.2f", distanceToTarget));
			} else {
				distanceToTarget = actualTargetWidth / (2.0 * Math.tan(imageTargetWidth * fovRadians / 2.0));
			}

		}
		return distanceToTarget;
	}

	@Deprecated
	public double getGearTargetStrafe() {
		// Use trig to return how far we need to strafe to center the target
		double strafeToTarget = 0;
		if (Robot.imageProcessor.targetsAcquired) {
			double fovRadians = (Math.PI / 180.0) * RobotMap.gearCamFov;
			// Based on sorting with farthest left at index [0] this gives pixel
			// dist
			// in our resolution agnostic -1 to 1 coordinate system; same as
			// (pixels width)/horiz_resolution
			// it's the average distance from zero (q1+q2)/2 (n) and then
			// normalized to two (R)
			double imageOffsetFromZero = 0.5 * (gearTargetX[1] + gearTargetX[0]) / 2.0;
			// The standard one that you get from Chief Delphi, etc, it
			// underestimates when you are far away
			// distanceToTarget = imageTargetWidth*actualTargetWidth / (2.0 *
			// Math.tan(fovRadians/2.0));
			// Derived from a circle, will slightly overestimate. Note the angle
			// is not divided by 2 here
			// distanceToTarget = imageTargetWidth*actualTargetWidth/fovRadians;
			// True formula, might as well use it
			strafeToTarget = this.getGearTargetDistance() * Math.tan(imageOffsetFromZero * fovRadians);
		}
		return strafeToTarget;
	}

	@SuppressWarnings("static-access")
	@Deprecated
	public double getTwistToTarget() {
		// Use average the areas of our two main targets to see if we need to
		// turn left or right
		double twistToTarget = 0;

		double areaRight = 0;
		double areaLeft = 0;
		double maxRightArea = 0;
		double maxLeftArea = 0;
		double minRightArea = 10000;
		double minLeftArea = 10000;
		double averages = 2.0;
		double averageDistance = 0;
		double averageStrafe = 0;

		Command capture[] = new Command[5];
		for (int i = 0; i < (int) averages; i++) {

			capture[i] = new ImageCaptureAndProcess();

			Timer.delay(0.05);
			areaLeft += (1 / averages) * Robot.imageProcessor.gearTargetAreas[0];
			areaRight += (1 / averages) * Robot.imageProcessor.gearTargetAreas[1];
			averageDistance += (1 / averages) * Robot.imageProcessor.getGearTargetDistance();
			averageStrafe += (1 / averages) * Robot.imageProcessor.getGearTargetStrafe();
			if (Robot.imageProcessor.gearTargetAreas[0] > maxLeftArea)
				maxLeftArea = Robot.imageProcessor.gearTargetAreas[0];
			if (Robot.imageProcessor.gearTargetAreas[1] > maxRightArea)
				maxRightArea = Robot.imageProcessor.gearTargetAreas[1];
			if (Robot.imageProcessor.gearTargetAreas[0] < minLeftArea)
				minLeftArea = Robot.imageProcessor.gearTargetAreas[0];
			if (Robot.imageProcessor.gearTargetAreas[1] < minRightArea)
				minRightArea = Robot.imageProcessor.gearTargetAreas[1];

		}

		System.out.println("Areas are: " + String.format("%.2f", areaLeft) + "±"
				+ String.format("%.2f", maxLeftArea - minLeftArea) + " and " + String.format("%.2f", areaRight) + "±"
				+ String.format("%.2f", maxRightArea - minRightArea));
		System.out.println("Average distance: " + String.format("%.2f", averageDistance) + "\t Average strafe: "
				+ String.format("%.2f", averageStrafe));

		// try to figure out how many degrees we are off
		return 1.0;
	}

}
