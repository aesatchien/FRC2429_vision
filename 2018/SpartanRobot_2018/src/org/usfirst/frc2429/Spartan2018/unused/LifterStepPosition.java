


package org.usfirst.frc2429.Spartan2018.unused;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc2429.Spartan2018.Robot;

/**
 *
 */
public class LifterStepPosition extends Command {

double direction;

    public LifterStepPosition() {
        requires(Robot.liftMechanism);
     }
    
    public LifterStepPosition(double direction) {
        requires(Robot.liftMechanism);
        this.direction=direction;
     }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	System.out.println("\nEntering LifterStepPosition at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    	
    	if (direction > 0) {
    		Robot.liftMechanism.stepPositionUp();
    	}
    	else {
    		Robot.liftMechanism.stepPositionDown();
    	}
    	
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
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
