package org.usfirst.frc2429.Spartan2018.subLifter;

import org.usfirst.frc2429.Spartan2018.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class LifterIncrementPosition extends InstantCommand {
	
	double direction;
    public LifterIncrementPosition() {
        super();
        requires(Robot.liftMechanism);
    }
    
    public LifterIncrementPosition(double direction) {
    	super();
        requires(Robot.liftMechanism);
        this.direction=direction;
     }

    // Called once when the command executes
    protected void initialize() {
    	System.out.println("\nEntering LifterIncrementPosition at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s");
    	
    	if (direction > 0) {
    		Robot.liftMechanism.incrementPositionUp();
    	}
    	else {
    		Robot.liftMechanism.incrementPositionDown();
    	}
    	
    }

}
