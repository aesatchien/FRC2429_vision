package org.usfirst.frc2429.Spartan2018;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousLogic {
	//------------------------------------------------------------------------------------

	private static final double OVERSHOOT = 4.0;
	//Scale (close) 
	private static final double SCALE_CLOSE = 308- OVERSHOOT;   // was 308.7- OVERSHOOT;
	private static final double SCALE_FINAL_APPROACH = 25.6;  // 20.6 final distance to drive to get arm to hit scale
	private static final double DRIVE_TO_VIEW_CUBE_FROM_SCALE = 40;
	//Switch (close)
	private static final double SWITCH_CLOSE = 150.6 - OVERSHOOT;
	private static final double SWITCH_FINAL_APPROACH = 17.9;  //  12.9 final distance to drive to get extended arm over switch
	private static final double DRIVE_TO_VIEW_CUBE_FROM_SWITCH = 75;
	//Scale (far)
	private static final double TO_SWIM_LANE = 211.2 - OVERSHOOT;
	private static final double ACROSS_SWIM_LANE = 195.1;
	private static final double FAR_SCALE_FINAL_APPROACH = 69.4;  //final distance to drive to get arm to hit scale
	private static final double DRIVE_TO_VIEW_CUBE_FROM_FAR_SCALE = 30;
	//Middle position
	private static final double PUSH_OFF_WALL = 30.0;
	//private static final double MIDDLE_DRIVE_TO_SWITCH = 80.7 - OVERSHOOT;  // Too far!
	private static final double MIDDLE_DRIVE_TO_SWITCH = 60.0;
	private static final double MIDDLE_FINAL_APPROACH = 21.7;  //final distance to drive to get extended arm over switch
	
	// Angles for all
	private static final double RIGHT_ANGLE = 83;  //going for 90 always overshoots
	private static final double MIDDLE_ANGLE = 42; //44.5 is the value from CAD
	private static final double CORRECTION_ANGLE = 20; //44.5 is the value from CAD
	private static final double BACK_AWAY = -21.0;
	
	// angleMultiplier allows us to cut the programming in half from mirror symmetry 
	private static double angleMultiplier;
	
	//Mathmatical representations of position - {1,2,3} -> {left, middle, right}
	public static int scalePos;
	public static int switchPos;
	public static int robotPos;
	//public static int autonomousPosition;
	public static int autonomousAbort;

	/**
	 *  initialDrop[] carries the first move to the target [0] and optional turns and moves if necessary
	 */
	public static double[] initialDrop;
	/**
	 *  afterDrop[] contains the turn angle [0] and distance [1] etc to get lined up on a cube 
	 */
	public static double[] afterDrop;
	/**
	 *  turnToDrop is the final angle to turn before ejecting the cube
	 */
	public static double turnToDrop;
	public static double driveToDrop;
	public static double backAway=BACK_AWAY;
	
	
	//In case encoders broken, etc
	public static boolean bAutonomousAbort = false;
	// Tells us if we have to cross the swim lane
	public static boolean bSecondMovement;
	// Tells us how high to lift the arm
	public static boolean bTargetScale;
	public static boolean bPickupCube = true;

	//------------------------------------------------------------------------------------

	public AutonomousLogic() {
		initialDrop = new double[5];	
		afterDrop = new double[3];
	}
	
	public static void getDashboard() {
		//Read where we put the robot based on what was entered into the dashboard
		robotPos = (int) (0.45 + SmartDashboard.getNumber("AutoPosition", 3) );  // Just in case the double rounds to 2.999 etc
		autonomousAbort = (int) (0.45 + SmartDashboard.getNumber("AutoAbort", 0) );  // Just in case the double rounds to 0.999 etc
    	System.out.println("Robot position is: " + String.format("%.1f",(double) robotPos));
    	System.out.println("Choice to abort is: " + String.format("%.1f",(double) autonomousAbort));
		
    	//Define all angles as if we are on the right side.  Your turns are reversed if on the left
    	if (robotPos == 1) {
    		angleMultiplier = 1.0;
    	} else if (robotPos == 3) {
    		angleMultiplier = -1.0;
    	}
    	
    	if (autonomousAbort == 0) {
    		bAutonomousAbort = false;
    		System.out.println("**AUTONOMOUS IS A GO** " + String.format("%.1f",(double) autonomousAbort));
    	} else if (autonomousAbort == 1) {
    		bAutonomousAbort = true;
    		System.out.println("**ABORTING AUTONOMOUS** " + String.format("%.1f",(double) autonomousAbort));
    	}
    	
	}
	
	public static void analyzeFieldMessage(String fieldMessage) {
		if(fieldMessage.charAt(0) == 'L') {
			switchPos = 1;
		}else if(fieldMessage.charAt(0) == 'R') {
			switchPos = 3;
		}
		
		if(fieldMessage.charAt(1) == 'L') {
			scalePos = 1;
		}else if(fieldMessage.charAt(1) == 'R') {
			scalePos = 3;
		}
		System.out.println("Switch position is: " + String.format("%.1f",(double) switchPos));
		System.out.println("Scale position is: " + String.format("%.1f",(double) scalePos));
	}
	
	public static int deltaPos(int robotPos, int fieldObject) {
		return Math.abs(robotPos - fieldObject);
	}
	
	public static void populateArrays() {
		if(deltaPos(robotPos, scalePos) == 0) {
			//Going for scale on same side
			System.out.println("Going for close scale at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s...");
			bSecondMovement = false;
			bTargetScale = true;
			initialDrop[0] = SCALE_CLOSE;
			turnToDrop = RIGHT_ANGLE * angleMultiplier;
			driveToDrop =  SCALE_FINAL_APPROACH;
			//Turn back to face the way we came
			afterDrop[0] = RIGHT_ANGLE * angleMultiplier;  //face back to the alliance
			afterDrop[1] = DRIVE_TO_VIEW_CUBE_FROM_SCALE;  // forward until we have a good view of the cube
			afterDrop[2] = -1.0*CORRECTION_ANGLE*angleMultiplier;
		}else if(deltaPos(robotPos, switchPos) == 0) {
			// Going for switch on same side
			bSecondMovement = false;
			bTargetScale = false;
			System.out.println("Going for close switch at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s...");
			initialDrop[0] = SWITCH_CLOSE;
			turnToDrop = RIGHT_ANGLE * angleMultiplier;
			driveToDrop =  SWITCH_FINAL_APPROACH;
			backAway = -12.0;
			//Get us back to facing the cubes
			afterDrop[0] = RIGHT_ANGLE * angleMultiplier;  //face back to the alliance
			afterDrop[1] = -1.0*DRIVE_TO_VIEW_CUBE_FROM_SWITCH;  // back up until we have a good view of the cube
			afterDrop[2] = -1.0*CORRECTION_ANGLE*angleMultiplier;

		}else if(deltaPos(robotPos, scalePos) == 2) {
			//Going for scale on opposite side
			System.out.println("Going for FAR scale at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s...");
			bSecondMovement = true;
			bTargetScale = true;
			initialDrop[0] = TO_SWIM_LANE;
			initialDrop[1] = RIGHT_ANGLE * angleMultiplier;
			initialDrop[2] = ACROSS_SWIM_LANE;
			turnToDrop = -1.0* RIGHT_ANGLE * angleMultiplier;
			driveToDrop =  FAR_SCALE_FINAL_APPROACH;
			//Get us back to facing the cubes
			//afterDrop[0] = 2.0* RIGHT_ANGLE;  // do a 180 to face back to the alliance
			afterDrop[0] = RIGHT_ANGLE * angleMultiplier;  // Seems to be calling the 180 for some reason in competition
			afterDrop[1] = DRIVE_TO_VIEW_CUBE_FROM_FAR_SCALE ;  // probably already facing a cube
			afterDrop[2] = CORRECTION_ANGLE*angleMultiplier;
			
			//----------------------------------------------------------
			// Decided to abort this
			bAutonomousAbort = true;

		}else if(deltaPos(robotPos, switchPos) == 1){
			//Going for switch from middle (POS 2)
			if(switchPos==1) {
				//Left turn
				angleMultiplier = -1.0;
				System.out.println("Going for LEFT switch from middle at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s...");
			}else {
				//Right turn
				angleMultiplier = 1.0;
				System.out.println("Going for RIGHT switch from middle at " + String.format("%.2f",Timer.getFPGATimestamp()) + "s...");
			}
			bSecondMovement = true;
			bTargetScale = false;
			initialDrop[0] = PUSH_OFF_WALL;
			initialDrop[1] = MIDDLE_ANGLE * angleMultiplier;
			initialDrop[2] = MIDDLE_DRIVE_TO_SWITCH;
			turnToDrop = -1.0*MIDDLE_ANGLE * angleMultiplier;
			driveToDrop =  MIDDLE_FINAL_APPROACH;
			
			//Get us back to facing the cubes
			afterDrop[0] = MIDDLE_ANGLE * angleMultiplier;
			afterDrop[1] = -1.0*MIDDLE_DRIVE_TO_SWITCH;
			afterDrop[2] = -1.0*MIDDLE_ANGLE * angleMultiplier;
			
		}// if on the deltaPos
			
	}// end of populateArrays
	

	//Getters for all of the variables needed for constructing autonomous
	public boolean isSecondMovement() {
		return bSecondMovement;
	}
	public boolean isTargetScale() {
		return bTargetScale;
	}
	public boolean isAutonomousAbort() {
		return bAutonomousAbort;
	}
	public boolean isPickupCube() {
		return bPickupCube;
	}
}
