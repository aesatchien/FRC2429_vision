package org.usfirst.frc2429.Spartan2018.autonomous;

import org.usfirst.frc2429.Spartan2018.AutonomousLogic;
import org.usfirst.frc2429.Spartan2018.Robot;
import org.usfirst.frc2429.Spartan2018.subLifter.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;


/**
 *  The autonomous commandgroup that is made after setting all the logic for the 
 */
public class AutonomousTest extends CommandGroup {

	double timeOut= 4.0;
	double pause = 0.3;
    @SuppressWarnings("static-access")
	public AutonomousTest() {
    	
		//We told it to abort autonomous - just drive across the line
    	if (Robot.autoLogic.isAutonomousAbort()) {
    		System.out.println("Aborting Autonomous at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");
    		//Move forward for 2 seconds to cross the 10' line
    		addSequential(new TimedMove(0.7, 2.5),timeOut);
//------------------------------------------------------------------------------------
    		
    	}else {
			// 1: Drive to initial position
			System.out.println("Initial move forward of "+ String.format("%.2f",Robot.autoLogic.initialDrop[0]) +" at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");
			addSequential(new DriveRobotPID(Robot.autoLogic.initialDrop[0]),timeOut);
			addSequential(new WaitCommand(pause));
			// 2: Decide to cross swim lane or line up from middle position
			if (Robot.autoLogic.isSecondMovement()) {
			  System.out.println("Requiring additional moves at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");
			  System.out.println("Initial rotation of "+ String.format("%.2f",Robot.autoLogic.initialDrop[1]) +" at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");
			  addSequential(new TurnRobotPID(Robot.autoLogic.initialDrop[1], false),timeOut);
			  addSequential(new WaitCommand(pause));
			  System.out.println("Second move forward of "+ String.format("%.2f",Robot.autoLogic.initialDrop[2]) +" at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");
			  addSequential(new DriveRobotPID(Robot.autoLogic.initialDrop[2]),timeOut);
			  addSequential(new WaitCommand(pause));
			} else {
				System.out.println("Single initial move completed at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");
			}
			// 3: Decide how high to go with the arm - first call to lifter
			if (Robot.autoLogic.isTargetScale()) {
				System.out.println("Lifting arm to scale at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");
				addSequential(new LifterTop(),timeOut);
				addSequential(new WaitCommand(pause));
			} else {
				System.out.println("Lifting arm to switch at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");
				addSequential(new LifterSwitch(),timeOut);
				//addSequential(new LifterSetHeight(3900),timeOut);
				//Default pause is not enough for this one, apparently
				addSequential(new WaitCommand(pause));
			}
			
			// 4: Make the final turn before we drop
			System.out.println("Turning to drop target with turn of "+ String.format("%.2f",Robot.autoLogic.turnToDrop) +"degrees at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");	
			addSequential(new TurnRobotPID(Robot.autoLogic.turnToDrop, false),timeOut);
			addSequential(new WaitCommand(pause));
			//We may need some sort of sensor to get us to stop properly ... don't want to tip over, but need to touch scale
			// 5: So we could just inch forward or coast into it with timed move forward
			System.out.println("Creeping forward to target with "+ String.format("%.2f",Robot.autoLogic.driveToDrop) +"inches at" + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");
			//addSequential(new TimedMove(0.25, 1));
			addSequential(new DriveRobotPID(Robot.autoLogic.driveToDrop,0.25,3.0),2.0);
			addSequential(new WaitCommand(pause));
			// 6: Eject the cube - this is the first call to the intake subsystem 
			System.out.println("Ejecting cube at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");	
			addSequential(new EjectCube(0.75),timeOut);
			addSequential(new WaitCommand(pause));
			// 7: Turn back to face cubes so we can safely lower the arm
			System.out.println("Backing away from target of " + String.format("%.2f",Robot.autoLogic.backAway) + " inches at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");	
			addSequential(new DriveRobotPID(Robot.autoLogic.backAway),timeOut);
			addSequential(new WaitCommand(pause));
			//addSequential(new TimedMove(-0.4, 1));
			// 8: turn to the cubes
			System.out.println("Turning to drop arm with " + String.format("%.2f",Robot.autoLogic.afterDrop[0]) + " degrees at "+ String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");	
			addSequential(new TurnRobotPID(Robot.autoLogic.afterDrop[0], false),timeOut);
			// 9: drop the arm
			System.out.println("Lowering arm at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");	
			addSequential(new LifterBottom(),timeOut);
			addSequential(new WaitCommand(pause));
			System.out.println("Driving to see cubes with " + String.format("%.2f",Robot.autoLogic.afterDrop[1]) + " inches at " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");	
			// 10: drive to where we can see the cubes
			//addSequential(new DriveRobotPID(Robot.autoLogic.afterDrop[1]),timeOut);
			// 11: turn to the cubes if necessary
			System.out.println("Turning to focus on cube " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");	
			//addSequential(new TurnRobotPID(Robot.autoLogic.afterDrop[2], false),timeOut);
			addSequential(new WaitCommand(pause));
			if (Robot.autoLogic.isPickupCube()) {
				// 12: pick up the cube
				System.out
						.println("Picking up a cube at " + String.format("%.2f", Timer.getFPGATimestamp()-Robot.enabledTime) + "s...");
				addSequential(new WaitCommand(pause));
				//addSequential(new PickupCube(),timeOut);
			} //end on pickup cube
		} //end on autonomousAbort
		
		System.out.println("Finished creating autonomous after  " + String.format("%.2f",Timer.getFPGATimestamp()-Robot.enabledTime) + "s.");
    } 
}
