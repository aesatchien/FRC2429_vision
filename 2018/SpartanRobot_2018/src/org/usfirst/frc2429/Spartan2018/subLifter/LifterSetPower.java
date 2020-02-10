package org.usfirst.frc2429.Spartan2018.subLifter;

import org.usfirst.frc2429.Spartan2018.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LifterSetPower extends Command {
	double power =-0.30;
    public LifterSetPower() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("\nEntering LifterWithStick at " + String.format("%.2f",Timer.getFPGATimestamp()- Robot.enabledTime) + "s");
    	Robot.liftMechanism.setArmMoving(true);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	power =  SmartDashboard.getNumber("Set Power", 0);
    	Robot.liftMechanism.setOutput(power);
    	//Timer.delay(1);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("{" + String.format("%.3f",Robot.liftMechanism.getPosition()) + ","
    + String.format("%.3f",Robot.liftMechanism.getPower()) + "}, ");
    	Robot.liftMechanism.setArmMoving(false);
    	//Robot.liftMechanism.reset();
    	//Robot.liftMechanism.holdPosition();
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("{" + String.format("%.3f",Robot.liftMechanism.getPosition()) + ","
    + String.format("%.3f",Robot.liftMechanism.getPower()) + "}, ");
    	Robot.liftMechanism.setArmMoving(false);
    	//Robot.liftMechanism.reset();
    	//Robot.liftMechanism.holdPosition();
    }
}
