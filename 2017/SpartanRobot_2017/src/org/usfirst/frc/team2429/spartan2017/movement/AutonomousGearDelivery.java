package org.usfirst.frc.team2429.spartan2017.movement;

import org.usfirst.frc.team2429.spartan2017.commands.*;


import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousGearDelivery extends CommandGroup {

    public AutonomousGearDelivery() {

    	//Rotate is no good now that we had to switch to the Genius cam... too bad
    	
    	//Step 1: center on the gear, using the live image (true)
		addSequential(new RobotCenterOnGearPID(true),3);
		//Take a snapshot of the imaging variables
		//since fixing one of the first two tends to lose the image of the target
		//addSequential(new AutonomousSetImagingVariables());
		//Step 4b Fix rotation
		// "true" means use image processing
		//addSequential(new RobotRotateCustomPID(0, true));
		//Step 4c then fix side to side using stored value (false option)
		//Which is actually moving straight since the gear is on the side of the robot
		//addSequential(new RobotCenterOnGearPID(false));
		//Stored value is never right, so fix it now that we are in the FOV
		//addSequential(new RobotCenterOnGearPID(true));
		//The final delivery, moving while maintaining heading and strafe - 3 seconds
		addSequential(new RobotDriveToGearTarget(),5);
    }
}
