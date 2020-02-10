package org.usfirst.frc2429.Spartan2018.unused;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc2429.Spartan2018.Robot;
/**
 * Move the lifter up using a PoV button
 */
public class LifterPOVUp extends Command {

    public LifterPOVUp() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.liftMechanism);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("\nEntering LifterPOVUp at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    	Robot.liftMechanism.stepPositionUp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.liftMechanism.movePositionUp();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !Robot.oi.driveButtonPovUp.get();
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.liftMechanism.stop();
    	System.out.println("Ending LifterPOVUp at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//Robot.liftMechanism.stop();
    	System.out.println("Interrupting LifterPOVUp at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    }
}
