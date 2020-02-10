
package org.usfirst.frc2429.Spartan2018.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc2429.Spartan2018.Robot;

/**
 *  This class corrects for drift when trying to drive straight
 */
public class GyroCorrectedTankDrive extends Command {
	
	private boolean bTwistActive;
	private double heading;
	private double correctedTwist;
	private double twistDeadZone = 0.05; // Minimum value for a twist joystick before we react to it
	private double XYDeadZone = 0.2;     //Minimum value for XY before we try to maintain a heading
	private double leadTime = 0.2;      //Try to anticipate where the gyro will end based on overcorrection
	private int executionCount;
	
	// Added these to include a derivative term in the gyro correction
	double previousTwistError = 0;		//Use this for a kD on the gyro
	double gyroError = 0;
	
    public GyroCorrectedTankDrive() {
        requires(Robot.drivetrain);
   }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	System.out.println("\nEntering GyroCorrected Tank Drive at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    	bTwistActive = false;
    	heading = Robot.drivetrain.driveGyro.getAngle();
    	executionCount = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {	
    	
    	double curJoyTwist = Robot.oi.getTwist();  //only read this once per pass
    	gyroError = (heading - Robot.drivetrain.driveGyro.getAngle());
    	// Correct for heading if we aren't actively using the twist  
    	if(!bTwistActive && Math.abs(curJoyTwist) < twistDeadZone){
    		// case where we're doing nothing with the twist but still driving
    		if (Math.sqrt( Math.pow(Robot.oi.getThrust(),2) ) > XYDeadZone){
    			//actually moving in X or Y
    			correctedTwist = Robot.drivetrain.kPdriveGyro*(gyroError)  +
    					Robot.drivetrain.kDdriveGyro*(gyroError-previousTwistError);
    		} else {
    			// just sitting still
    			correctedTwist = 0;
    		}
    		
    	}else if(bTwistActive && Math.abs(curJoyTwist) < twistDeadZone){
    		// case where we were twisting last time but not this time
    		bTwistActive = false;
    		heading = Robot.drivetrain.driveGyro.getAngle() + Robot.drivetrain.driveGyro.getRate() * leadTime;
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
    	previousTwistError = heading - Robot.drivetrain.driveGyro.getAngle();
    	executionCount++;
//------------------------------------------------    	
    	if (Robot.drivetrain.getJoystickEnabled()) {
    		Robot.drivetrain.smoothDrive(- Robot.oi.getThrust(),correctedTwist);
    	} else {
    		Robot.drivetrain.smoothDrive(0,0);
    	}
    	}
    
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    	Robot.drivetrain.drive(0, 0);
    	System.out.println("\nEnding GyroCorrected Tank Drive at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	Robot.drivetrain.drive(0, 0);
    	System.out.println("Interrupting GyroCorrected Tank Drive at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    }
}
