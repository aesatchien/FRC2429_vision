package org.usfirst.frc.team2429.spartan2017.movement;

import org.usfirst.frc.team2429.spartan2017.Robot;
import org.usfirst.frc.team2429.spartan2017.ballcommands.BallShooterStart;
import org.usfirst.frc.team2429.spartan2017.commands.*;
import org.usfirst.frc.team2429.spartan2017.movement.*;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousShooting extends CommandGroup {

    public AutonomousShooting() {
    	addSequential(new AutonomousSetImagingVariables());
    	addSequential(new RobotRotateCustomPID(0, true),2);
    	addSequential(new SitStill(0.1));
    	addSequential(new AutonomousSetImagingVariables());
    	//I feel like we should start shooting early, before we do the final movement adjust
    	addSequential(new BallShooterStart());
    	addSequential(new RobotDriveToShoot(),4);
      	addSequential(new SitStill(0.1));
      	addSequential(new AutonomousSetImagingVariables());
      	addSequential(new RobotRotateCustomPID(0, true),2);
      	//Not sure if I want this earlier or later
      	//addSequential(new BallShooterStart());
    }
}
