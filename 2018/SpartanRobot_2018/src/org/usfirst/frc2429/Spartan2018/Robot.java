package org.usfirst.frc2429.Spartan2018;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc2429.Spartan2018.autonomous.AutonomousTest;
import org.usfirst.frc2429.Spartan2018.commands.*;
import org.usfirst.frc2429.Spartan2018.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {
	//------------------------------------------------------------------------------------

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    public static OI oi;
    public static ImageProcessor imageProcessor;
    public static AutonomousLogic autoLogic;
    public static Climber climber;
    public static Drivetrain drivetrain;
    public static Intake intake;
    public static LiftMechanism liftMechanism;
    public static DriverStation ds;
    private static String fieldPlates;
    
    public static boolean bDebugging = true;
    public static double enabledTime = 0;


	//------------------------------------------------------------------------------------

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        climber = new Climber();
        drivetrain = new Drivetrain();
        intake = new Intake();
        liftMechanism = new LiftMechanism();
        imageProcessor = new ImageProcessor();
        autoLogic = new AutonomousLogic();

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();
        ds = DriverStation.getInstance();

		// Show what command your subsystem is running on the SmartDashboard
		SmartDashboard.putData(drivetrain);
		SmartDashboard.putData(liftMechanism);
		SmartDashboard.putData(intake);
		reset();   
        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
//        chooser.addDefault("Autonomous Command", new AutonomousCommand());
//        SmartDashboard.putData("Auto mode", chooser);
    }
	//------------------------------------------------------------------------------------

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){
    	
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        log();
    }

    @Override
    public void autonomousInit() {
    	enabledTime = Timer.getFPGATimestamp();
    	fieldPlates = ds.getGameSpecificMessage();
    	//test the field message
    	autoLogic.analyzeFieldMessage(fieldPlates);
    	//analyze the dashboard message
    	autoLogic.getDashboard();
    	//set up all the distances
    	autoLogic.populateArrays();
    	//set up the autonomous procedure
    	autonomousCommand = new AutonomousTest();
        if (autonomousCommand != null) autonomousCommand.start();
        reset();
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        log();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
    	enabledTime = Timer.getFPGATimestamp();
        if (autonomousCommand != null) autonomousCommand.cancel();
        Scheduler.getInstance().removeAll();
        reset();
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        log();
    }
    
	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */
	private void log() {
		drivetrain.log();
		intake.log();
		liftMechanism.log();
		oi.log();
		imageProcessor.log();
		
	}
	private void reset() {
		drivetrain.reset();
		liftMechanism.reset();
	}
}
