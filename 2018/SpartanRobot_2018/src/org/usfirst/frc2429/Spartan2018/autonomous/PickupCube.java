package org.usfirst.frc2429.Spartan2018.autonomous;

import org.usfirst.frc2429.Spartan2018.AutonomousLogic;
import org.usfirst.frc2429.Spartan2018.subIntake.IntakeStart;
import org.usfirst.frc2429.Spartan2018.subIntake.IntakeHold;
import org.usfirst.frc2429.Spartan2018.subLifter.LifterSetHeight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;


/**
 *
 */
public class PickupCube extends CommandGroup {
	static double pause = 0.1;
    public PickupCube() {
    	
    	//Drive to initial position

    	addSequential(new WaitCommand(pause));
    	addParallel(new IntakeStart());
    	addSequential(new WaitCommand(pause));
    	addSequential(new TurnRobotPID(true));  	
    	addSequential(new WaitCommand(pause));
    	addSequential(new DriveRobotPID(true));
    	addSequential(new WaitCommand(pause));
    	addSequential(new IntakeHold());  
    	
    }
}
