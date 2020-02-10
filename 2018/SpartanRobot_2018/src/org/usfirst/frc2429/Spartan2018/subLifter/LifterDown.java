package org.usfirst.frc2429.Spartan2018.subLifter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc2429.Spartan2018.Robot;
/**
 *  Command moves the lifter down a set amount
 */
public class LifterDown extends Command {

    public LifterDown() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.liftMechanism);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("\nEntering LifterDown at " + String.format("%.2f",Timer.getFPGATimestamp()- Robot.enabledTime) + "s");
    	Robot.liftMechanism.setArmMoving(true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.liftMechanism.moveDown();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("Ending LifterDown at " + String.format("%.2f",Timer.getFPGATimestamp()- Robot.enabledTime) + "s");
    	Robot.liftMechanism.setArmMoving(false);
    	//Robot.liftMechanism.setPosition(Robot.liftMechanism.getPosition());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("Interrupting LifterDown at " + String.format("%.2f",Timer.getFPGATimestamp()- Robot.enabledTime) + "s");
    	Robot.liftMechanism.setArmMoving(false);
    	//Robot.liftMechanism.setPosition(Robot.liftMechanism.getPosition());
    }
}
