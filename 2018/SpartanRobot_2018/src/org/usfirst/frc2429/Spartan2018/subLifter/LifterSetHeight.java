


package org.usfirst.frc2429.Spartan2018.subLifter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc2429.Spartan2018.Robot;

/**
 *
 */
public class LifterSetHeight extends Command {

double height;

    public LifterSetHeight() {
        requires(Robot.liftMechanism);
     }
    
    public LifterSetHeight(double height) {
        requires(Robot.liftMechanism);
        this.height=height;
     }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	System.out.println("LifterSetHeight called for position: " + String.format("%.2f",height) + " at " + String.format("%.2f", Timer.getFPGATimestamp()- Robot.enabledTime)  + "s");
    	Robot.liftMechanism.setPosition(height);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    	System.out.println("LifterSetHeight ended at: " + String.format("%.2f", Timer.getFPGATimestamp()- Robot.enabledTime) + "s");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	System.out.println("LifterSetHeight interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()- Robot.enabledTime) + "s");
    }
}
