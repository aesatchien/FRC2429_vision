package org.usfirst.frc.team2429.spartan2017.deprecated;

import org.usfirst.frc.team2429.spartan2017.OI;
import org.usfirst.frc.team2429.spartan2017.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CameraBrightnessUp extends Command {

    public CameraBrightnessUp() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	RobotMap.gearCameraBrightness+=5;
    	RobotMap.gearCam.setBrightness(RobotMap.gearCameraBrightness);
    	SmartDashboard.putNumber("Camera Brightness: ", RobotMap.gearCameraBrightness);
    	System.out.println("CameraBrightness raised to "+RobotMap.gearCameraBrightness+" at: " + String.format("%.2f",Timer.getFPGATimestamp())+"s");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
