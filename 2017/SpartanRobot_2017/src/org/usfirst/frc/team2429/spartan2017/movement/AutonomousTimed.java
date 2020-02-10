package org.usfirst.frc.team2429.spartan2017.movement;

import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc.team2429.spartan2017.commands.*;
import org.usfirst.frc.team2429.spartan2017.movement.*;


/**
 *
 */
public class AutonomousTimed extends CommandGroup {

    public AutonomousTimed() {
    //	addSequential(new RobotMoveForward(0.5));
    	//addSequential(new RobotStrafeRight(2.0));
    	addSequential(new RobotMoveForward(4.2));
    }
}
