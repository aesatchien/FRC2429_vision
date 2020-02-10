package org.usfirst.frc2429.Spartan2018.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import org.usfirst.frc2429.Spartan2018.Robot;

/**
 *  Call this for as many seconds as you need
 */
public class EjectCube extends TimedCommand {
	static double timeout;
    public EjectCube(double timeout) {
        super(timeout);
        this.timeout=timeout;
        requires (Robot.intake);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("EjectCube called for: " + String.format("%.2f", timeout) + " seconds at "  + String.format("%.2f", Timer.getFPGATimestamp()- Robot.enabledTime)  + "s");
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intake.intakeReverse();
    }

    // Called once after timeout
    protected void end() {
    	System.out.println("EjectCube ended at: " + String.format("%.2f", Timer.getFPGATimestamp()- Robot.enabledTime) + "s");
    	Robot.intake.intakeStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("EjectCube interrupted at: " + String.format("%.2f", Timer.getFPGATimestamp()- Robot.enabledTime) + "s");
    	Robot.intake.intakeStop();
    }
}
