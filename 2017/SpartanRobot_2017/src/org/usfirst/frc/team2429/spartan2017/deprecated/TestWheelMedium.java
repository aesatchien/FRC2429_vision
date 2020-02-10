package org.usfirst.frc.team2429.spartan2017.deprecated;

import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TestWheelMedium extends Command {

	private double testPower;
	
    public TestWheelMedium() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(30.0);
    	testPower = 0.35;
		System.out.println("TestWheelMedium called at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s");
    	System.out.println("Testing with: "  + testPower);

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	RobotMap.drivetrainrearLeftChannel.set(testPower);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return isTimedOut();    
    	
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("TestWheelMedium ended at: " + String.format("%.2f", Timer.getFPGATimestamp()) + "s   after running " + String.format("%.2f", timeSinceInitialized()) + "s" );
    	RobotMap.drivetrainrearLeftChannel.set(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
