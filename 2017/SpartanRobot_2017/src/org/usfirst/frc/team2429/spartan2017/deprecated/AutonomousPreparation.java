package org.usfirst.frc.team2429.spartan2017.deprecated;

import org.usfirst.frc.team2429.spartan2017.Robot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutonomousPreparation extends Command {

    public AutonomousPreparation() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    //You want this to do the exact same thing the one in the Robot.init() does
    //So you can debug it w/o rebooting the roboRIO
    protected void initialize() {
    	
    	System.out.println("\nAutonomousPreparation called at: " + String.format("%.2f",Timer.getFPGATimestamp())+"s");
    	switch(DriverStation.getInstance().getLocation()) {
    	case 1: 
				Robot.oi.setAutonomousLocation("LEFT");
				Robot.oi.setAutonomousStartingDistance(1.0);
				Robot.oi.setAutonomousGearRotationAngle(10.0);
				break;
		case 2: 
				Robot.oi.setAutonomousLocation("CENTER");
				Robot.oi.setAutonomousStartingDistance(2.0);
				Robot.oi.setAutonomousGearRotationAngle(20.0);
				break;
		case 3:     				
				Robot.oi.setAutonomousLocation("RIGHT");
				Robot.oi.setAutonomousStartingDistance(3.0);
				Robot.oi.setAutonomousGearRotationAngle(30.0);	
				break;
    		default: //RobotMap.autonomousLocation = "error";
    				break;
       	}
    	SmartDashboard.putString("autoLocation", Robot.oi.getAutonomousLocation());
		SmartDashboard.putNumber("autoDistance", Robot.oi.getAutonomousStartingDistance());
		SmartDashboard.putNumber("autoRotation", Robot.oi.getAutonomousGearRotationAngle());
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
    	System.out.println("With Parameters location: " + Robot.oi.getAutonomousLocation() + ", start distance: "+ Robot.oi.getAutonomousStartingDistance() +
    			", and gear angle: " + Robot.oi.getAutonomousGearRotationAngle());
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
