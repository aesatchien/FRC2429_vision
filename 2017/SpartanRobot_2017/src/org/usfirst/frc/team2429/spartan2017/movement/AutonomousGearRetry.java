package org.usfirst.frc.team2429.spartan2017.movement;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *  Copies GearDelivery but checks for a gear first; if none we just cross the baseline
 */
public class AutonomousGearRetry extends Command {

	public boolean isOperatorEnabled;
	double heading;
	private int executionCount;
	
	private boolean bGearPresent;
		
	// At the moment we will stop when 15 inches from target - this can change once we get the camera mounted
	double dropDistance = 8;
	//We'll quit if the setpoint is within kTolerance
	double kTolerance = 0.5;
	double previousStrafeError = 0;	
	// Correction terms
	double xMotion;
	double yMotion;
	double twist;
	// May not need a minimum speed because we're stopping 15" away - kP pulls us in
	double minimumSpeed = 0.18;
	//Have to make sure it actually starts, so make sure it lasts at least 1 second
	double minumumTime = 0.1;
	private NetworkTable gearTable;
	
	// TODO - add a version where this is driven by a button
	public AutonomousGearRetry() {

		isOperatorEnabled = true;
		bGearPresent = true;
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drivetrain);
		
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		System.out.println("AutonomousGearRetry called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
    	SmartDashboard.putNumber("Parameter", Robot.imageProcessor.gearTargetDistance);
		
    	// If the voltage on the gear sensor is less than a volt, there is a gear present
    	bGearPresent = (RobotMap.gearUltra.getVoltage() < 1.0) ? true : false ;
    	
    	if (bGearPresent){
    		int counter = 0;
    		gearTable = NetworkTable.getTable("GearCam");
    		Robot.imageProcessor.gearTargetCount = gearTable.getNumber("targets", -1.0);
    		
    		while (  (counter < 5) && (Robot.imageProcessor.gearTargetCount < 2) ){
    			Robot.imageProcessor.gearTargetCount = gearTable.getNumber("targets", -1.0);
    			Robot.drivetrain.linearMecanum(0, 0, 0);
    			Timer.delay(0.05);
    			counter++;
    		}
    		System.out.println("Exited Loop After: " + counter*0.05 + " seconds" );
    		setTimeout(6);
    		previousStrafeError = Robot.imageProcessor.gearTargetStrafe;	
    	}
    	else{
    		setTimeout(3.0);
    	}
    	
    		
		//don't let it rotate off the starting orientation 
		RobotMap.gyro.reset();
		heading = RobotMap.gyro.getAngle();
		executionCount = 0;
    	SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
    	SmartDashboard.putNumber("Parameter", Robot.imageProcessor.gearTargetDistance);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (bGearPresent){
			//xMotion is the strafe that we perform moving in to the gear
			xMotion = Math.min(
					Robot.drivetrain.autonomousGearDeliverySpeed,
					(Robot.imageProcessor.gearTargetDistance * Robot.drivetrain.kStrafeProportional)+ minimumSpeed);
			
			//twist corrects for the robot rotating while we are moving in, same as everywhere else
			twist = (heading - RobotMap.gyro.getAngle()) * Robot.drivetrain.kGyroProportional;
			
			//yMotion corrects for us getting off center while moving forward
			// should be continuously set by SensorUpdate()  
			// added a derivative term so we could really hold the center better
			yMotion =  Math.min(0.1, Robot.imageProcessor.gearTargetStrafe * Robot.drivetrain.kGearDeliveryForwardProportional) + 
					Robot.drivetrain.kGearDeliveryForwardDerivative * (Robot.imageProcessor.gearTargetStrafe - previousStrafeError);
			
			// Drive on in: negative x moves robot the direction the camera is facing
			// and have to flip y because the image processor gives negative values when we need to move backwards 
			// and negative y moves the robot forwards.  Sorry about the complication
			RobotMap.drivetrainRobot.mecanumDrive_Cartesian(-xMotion, -yMotion, twist, 0);
			previousStrafeError = Robot.imageProcessor.gearTargetStrafe;	
		}
		else{
			//Just cross the baseline - but figure out if the robot is facing forward or backward
			//twist corrects for the robot rotating while we are moving in, same as everywhere else
			twist = (heading - RobotMap.gyro.getAngle()) * Robot.drivetrain.kGyroProportional;
			int position = (int) (0.2 + SmartDashboard.getNumber("Autonomous Position", 3) );
			double speed = (position == 3) ? (-1.0 * Robot.drivetrain.autonomousForwardSpeed) : Robot.drivetrain.autonomousForwardSpeed ;
			RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, speed, twist, 0);
		}
		
		
    	executionCount++;
    	SmartDashboard.putNumber("Iterations", executionCount);
    	SmartDashboard.putNumber("Error", Robot.imageProcessor.gearTargetDistance - dropDistance);
	}
	


	// Make this return true when this Command no longer needs to run execute()
	//Make sure we stay in this at least minimumTime so it has a chance to start
	protected boolean isFinished() {
		return (  isTimedOut() || ((timeSinceInitialized() > minumumTime) && ((Robot.imageProcessor.gearTargetDistance - dropDistance) < kTolerance)));
	}

	// Called once after isFinished returns true
	protected void end() {

		if (isOperatorEnabled) {
			System.out.println(
					"AutonomousGearRetry ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		} else {
			System.out.println("AutonomousGearRetry ended at: " + String.format("%.2f", Timer.getFPGATimestamp())
					+ "s   after running " + String.format("%.2f", timeSinceInitialized()) + "s");
		}
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		System.out.println(
				"AutonomousGearRetry interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
		RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0, 0, 0, 0);

	}
}
