package org.usfirst.frc.team2429.spartan2017.movement;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
/**
 *
 */
public class MecanumDrive extends Command {
	
	private boolean bTwistActive;
	private double heading;
	private double correctedTwist;
	private double twistDeadZone = 0.05; // Minimum value for a twist joystick before we react to it
	private double XYDeadZone = 0.2;     //Minimum value for XY before we try to maintain a heading
	private double leadTime = 0.2;      //Try to anticipate where the gyro will end based on overcorrection
	private int executionCount;
	
	// Added these two 02/19/2017 to include a derivative term in the gyro correction
	double previousTwistError = 0;		//Use this for a kD on the gyro
	double gyroError = 0;
	
    public MecanumDrive() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	bTwistActive = false;
    	heading = RobotMap.gyro.getAngle();
    	System.out.println("\nEntering Mecanum Drive at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    	executionCount = 0;
    	SmartDashboard.putString("Current Command", this.getClass().getSimpleName());
		SmartDashboard.putNumber("Parameter", 0);
		SmartDashboard.putNumber("Iterations", 0);
		SmartDashboard.putNumber("Error", 0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Changed on 1/31/17 to maintain heading; added main four conditions
    	//Updated 2/19/2017 to prevent it from maintaining heading while sitting still; split the corrected condition
    	//Should also keep the wheels from spinning when the robot is climbing the rope
    	//this should allow us to up the kGyroProportional to keep us driving straight... may want to add a derivative too 
    	
    	double curJoyTwist = Robot.oi.customGetJoyTwist();  //only read this once per pass
    	gyroError = (heading - RobotMap.gyro.getAngle());
    	
    	// Correct for heading if we aren't actively using the twist  
    	if(!bTwistActive && Math.abs(curJoyTwist) < twistDeadZone){
    		// case where we're doing nothing with the twist but still driving
    		if (Math.sqrt( Math.pow(Robot.oi.customGetJoyX(),2) + Math.pow(Robot.oi.customGetJoyY(),2)) > XYDeadZone){
    			//actually moving in X or Y
    			correctedTwist = Robot.drivetrain.kGyroProportional*(gyroError)  +
    					Robot.drivetrain.kGyroDerivative*(gyroError-previousTwistError);
    		} else {
    			// just sitting still
    			correctedTwist = 0;
    		}
    		
    	}else if(bTwistActive && Math.abs(curJoyTwist) < twistDeadZone){
    		// case where we were twisting last time but not this time
    		bTwistActive = false;
    		heading = RobotMap.gyro.getAngle() + RobotMap.gyro.getRate() * leadTime;
    		correctedTwist = 0;
    	}else if(!bTwistActive && Math.abs(curJoyTwist) > twistDeadZone){
    		// case where we were not twisting last time and started to this time
    		bTwistActive = true;
    		correctedTwist = curJoyTwist;
    	}else if(bTwistActive && Math.abs(curJoyTwist) > twistDeadZone){
    		// case where we were twisting last time and still are
    		correctedTwist = curJoyTwist;	
    	}else{
    		correctedTwist = curJoyTwist;
    	}
    	
    	
    	//RobotMap.drivetrainRobot.mecanumDrive_Cartesian(
    	//	Robot.oi.customGetJoyX(), Robot.oi.customGetJoyY(), Robot.oi.customGetJoyTwist() * OI.twistSensitivity, 0);
    	Robot.drivetrain.linearMecanum(
    		Robot.oi.customGetJoyX(), Robot.oi.customGetJoyY(), correctedTwist);
    	previousTwistError = heading - RobotMap.gyro.getAngle();
    	
    	executionCount++;
    	SmartDashboard.putNumber("Iterations", executionCount);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0,0,0,0);
    	System.out.println("Ending Mecanum Drive at " + String.format("%.2f",Timer.getFPGATimestamp())+"s");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	RobotMap.drivetrainRobot.mecanumDrive_Cartesian(0,0,0,0);
    	System.out.println("Interrupting Mecanum Drive at " + String.format("%.2f",Timer.getFPGATimestamp())+"s");
    }
}
