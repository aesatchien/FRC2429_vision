package org.usfirst.frc.team2429.spartan2017.ballcommands;

import org.usfirst.frc.team2429.spartan2017.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class BallAgitatorStop extends Command {

    public BallAgitatorStop() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.ballAgitator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("BallAgitatorStop Called");
    	if (Robot.ballAgitator.oscillateThread != null){
    		Robot.ballAgitator.oscillateThread.interrupt();
    	}
    	Timer.delay(0.02);
    	Robot.ballAgitator.stop();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
