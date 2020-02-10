package org.usfirst.frc.team2429.spartan2017.movement;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.ballcommands.BallShooterStart;
import org.usfirst.frc.team2429.spartan2017.commands.*;


import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousFullRoutine extends CommandGroup {

	public AutonomousFullRoutine() {

		final double waitTime = 0.1;  // 0.2 is more than enough in competition
		Robot.oi.setShootingAllowed(false);  // decided we're not going to shoot
		
		// Autonomous starting variables will be set in Robot.init() so they are safe to call here
		//Step 1: move forward the appropriate amount - 3s
		addSequential(new RobotMoveStraightPID(Robot.oi.getAutonomousStartingDistance()),4.0);
		//Step 2: rotate to gear - 1s
		addSequential(new RobotRotateCustomPID(Robot.oi.getAutonomousGearRotationAngle(), false), 3.0);
		
		// Angle determination with the gearcam is worthless; the GeniusCams are always misaligned at the factory so skip it
		//Step 3: center on the gear, using the live image (true)
		
		//Feel Like I should skip centering for time, but it may save time in the end
		addSequential(new RobotCenterOnGearPID(true), 1);
		//Steps 4abc are all optional - not really necessary if we drive up correctly
		// because DriveToGearTarget will fix minor misalignments
		//STEP 4 MAY HURT MORE THAN IT HELPS IF THE PI ROTATION IS BAD
		//Step 4a: Take a snapshot of the imaging variables
		//since fixing one of the first two tends to lose the image of the target
	//    	addSequential(new AutonomousSetImagingVariables());
		//Step 4b Fix rotation
		// "true" means use image processing
		//addSequential(new RobotRotateCustomPID(0, true));
		//Step 4c then fix side to side using stored value (false option)
		//Which is actually moving straight since the gear is on the side of the robot
		//addSequential(new RobotCenterOnGearPID(false));
		
		//The final delivery, moving while maintaining heading and strafe - 3 seconds plus 2 to sit for a total of 8 so far
		addSequential(new SitStill(waitTime));
	  	addSequential(new RobotDriveToGearTarget(), 4.0);
	  	addSequential(new SitStill(2.0));
		
		// Want two if conditions on shooting - one, we're in an allowable location and two we have it enabled
		if(Robot.oi.isAutonomousShootingPosition() && Robot.oi.isShootingAllowed()){
			//Need a delay so we can wait for the gear to be delivered.  Or a IsGearDelivered() command
			//so we can move on... wish we had put a limit switch in there
			//addSequential(new SitStill(2.0));
			//Move back to get ready to spin and shoot
			addSequential(new RobotStrafeRight(1.5));
			//Rotate and drive to where we think we need to be to start shooting
			addSequential(new RobotRotateCustomPID(Robot.oi.getAutonomousInitialShooterRotationAngle(), false),1.0);
			addSequential(new RobotMoveStraightPID(Robot.oi.getAutonomousShooterTravelDistance()),1.0);
			addSequential(new SitStill(.2));
			addSequential(new RobotRotateCustomPID(Robot.oi.getAutonomousSecondShooterRotationAngle(), false),2.0);
			addSequential(new SitStill(.1));
	    	addSequential(new AutonomousSetImagingVariables());
	    	//switch the rotate PID imaging true case to center on the hopper
	    	addSequential(new RobotRotateCustomPID(0, true),1.0);
	    	addSequential(new SitStill(.1));
	    	addSequential(new AutonomousSetImagingVariables());
	    	//start shooting - start early, may miss the first ball or two while we center it
	    	addSequential(new BallShooterStart());
	    	//drive up until we have our shot
	    	addSequential(new RobotDriveToShoot(),3.0);
	      	addSequential(new SitStill(.1));
	      	addSequential(new AutonomousSetImagingVariables());
	      	addSequential(new RobotRotateCustomPID(0, true), 2.0);
	      	
		} 
		
		//If you're not shooting, you may as well double check that the gear gets delivered
		else{
			//Try again on the gear delivery  takes us to 10s, then back in to 12s, and sitting for the rest.
			addSequential(new RobotStrafeRight(1.0));
			addSequential(new SitStill(waitTime));
			// addSequential(new RobotCenterOnGearPID(true));
			// addSequential(new SitStill(waitTime));
			//addSequential(new RobotDriveToGearTarget());
			addSequential(new AutonomousGearRetry());   //This one checks to see if we have a gear
			
		} 
		

		
	}
}
