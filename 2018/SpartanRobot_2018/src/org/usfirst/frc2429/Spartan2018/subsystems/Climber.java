// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2429.Spartan2018.subsystems;

import org.usfirst.frc2429.Spartan2018.commands.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Spark;


/**  
 *  Climbing subsystem
 */
public class Climber extends Subsystem {
	//------------------------------------------------------------------------------------

    private final SpeedController climberMotorLeft = new  VictorSP(4);
    private final SpeedController climberMotorRight = new VictorSP(5);
    private final double climbUpSpeed = 1.0;
    private final double climbDownSpeed = 0.25;
    private boolean climberOn = false;
	//------------------------------------------------------------------------------------

  //Constructor added 2/09/2018 CJH modeled after the GearsBot template - getting rid of RobotMap redundancy      
    public Climber() {
		super();
		SmartDashboard.putBoolean("Climber Motors", climberOn);
		
    }
	//------------------------------------------------------------------------------------

    @Override
    public void initDefaultCommand() {
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    public void climberToggle() {
    	//Used in command - IntakeToggle
    	if (!climberOn){
    		climberStart();
        	climberOn = true;
        }
    	else {
    		climberOn = false;
    		climberStop();
    	}
    	SmartDashboard.putBoolean("Climber Motors", climberOn);
    }
    public void climberReverseToggle() {
    	//Used in command - IntakeToggle
    	if (!climberOn){
    		climberReverse();
        	climberOn = true;
        }
    	else {
    		climberOn = false;
    		climberStop();
    	}
    	SmartDashboard.putBoolean("Climber Motors", climberOn);
    }
    public void climberStart() {
    	climberMotorLeft.set(climbUpSpeed);
    	climberMotorRight.set(climbUpSpeed);
    	climberOn = true;
    	SmartDashboard.putBoolean("Climber Motors", climberOn);
    }
    public void climberReverse() {
    	climberMotorLeft.set(-climbDownSpeed);
    	climberMotorRight.set(-climbDownSpeed);
    	climberOn = true;
    	SmartDashboard.putBoolean("Climber Motors", climberOn);
    }
    
    public void climberStop() {
    	climberMotorLeft.set(0);
    	climberMotorRight.set(0);
    	climberOn = false;
    	SmartDashboard.putBoolean("Climber Motors", climberOn);
    }
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}
