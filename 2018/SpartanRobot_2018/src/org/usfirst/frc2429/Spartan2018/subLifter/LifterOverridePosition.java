package org.usfirst.frc2429.Spartan2018.subLifter;

import org.usfirst.frc2429.Spartan2018.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class LifterOverridePosition extends Command {

    public LifterOverridePosition() {
        // Use requires() here to declare subsystem dependencies
    	 requires(Robot.liftMechanism);
    	 // eg. requires(chassis);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("\nCalling LifterOverridePosition at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    	Robot.liftMechanism.overrideLowerSetpoint(-6000);
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
    	Robot.liftMechanism.overrideLowerSetpoint(-1000);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.liftMechanism.overrideLowerSetpoint(-1000);
    }
}
