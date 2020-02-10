// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc.team2429.spartan2017.deprecated;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2429.spartan2017.Robot;


/**
 * We don't need this anymore.  All we want to do is oscillate
 */
@Deprecated
public class BallAgitatorReverse extends Command {

	public boolean isOscillate;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public BallAgitatorReverse() {
   	 requires(Robot.ballAgitator);
   	 isOscillate = false;
    }
    public BallAgitatorReverse(double timeout) {
      	 requires(Robot.ballAgitator);
      	 isOscillate = true;
      	 setTimeout(timeout);
       }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("BallAgitatorReverse Called");
    	Robot.ballAgitator.reverse();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
   
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (isOscillate){
    		return isTimedOut();
    	}else{
    		return true;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
