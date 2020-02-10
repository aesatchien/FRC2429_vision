package org.usfirst.frc2429.Spartan2018.subsystems;

import org.usfirst.frc2429.Spartan2018.OI;
import org.usfirst.frc2429.Spartan2018.Robot;
import org.usfirst.frc2429.Spartan2018.commands.*;
import org.usfirst.frc2429.Spartan2018.subLifter.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

/**
 *   This class defines the LiftMechanism subsystem
 */
public class LiftMechanism extends Subsystem {
	//------------------------------------------------------------------------------------
	// Limits
	private final double LIFTER_POWER_FORWARD_LIMIT = 0.7;
	private final double LIFTER_POWER_REVERSE_LIMIT = -0.65;
	private final double LIFTER_ENCODER_MAX = 5500;  // 5605 Competition 5578 Practice
	public final double LIFTER_SETPOINT_MAX = 5650; // was 6500 before gas springs
	//private final double LIFTER_SETPOINT_MIN = -1000;
	private double LIFTER_SETPOINT_MIN = -6000;
	
	//Max allowable difference between new setpoint and actual
	// Adding this seems to let us crank up kP and still not slam  3/31/2018
	private final double SETPOINT_ERROR_LIMIT = 500; // Max allowable difference between new setpoint and actual position
	
	//------------------------------------------------------------------------------------
	// Locations
	private final double LIFTER_BOTTOM = 0;
	private final double LIFTER_VAULT = 4000;
	private final double LIFTER_SWITCH = 4000;
	private final double LIFTER_TOP = 5700;
	private final double LIFTER_TOP_AUTONOMOUS = 5400;
	
	//------------------------------------------------------------------------------------
	//Multipliers
	private final double LIFTER_SLOW_VELOCITY = 5; 
	private final double JOYSTICK_POSITION_MULTIPLIER = 60;
	private final double JOYSTICK_VELOCITY_MULTIPLIER = 50;
	private final double JOYSTICK_RATE_UP = 40;  // Was 80 in competition.  Lowering to 75 in vegas
	private final double JOYSTICK_RATE_DOWN = 50;
	private final double POSITION_STEP_SIZE = 1000;

	
	private double modifiedOutput;
	//It's about 3000 units from top to bottom, and we update 50x per second 
	//So to get from bottom to top in two seconds at full joystick, we'd need to change by 30 per iteration 
	
	private final DigitalInput liftLimitHigh = new DigitalInput(9);
    private final DigitalInput liftArmContact = new DigitalInput(8);
	
    //private final Encoder liftEncoder = new Encoder(2, 3, false, EncodingType.k4X);
	private final WPI_TalonSRX liftTalonSRX_Master = new WPI_TalonSRX(1);
    private final WPI_TalonSRX liftTalonSRX_Slave = new WPI_TalonSRX(2);
    
    private static boolean bArmMoving = false;
    private static boolean bArmAtTop = false;
    private static boolean bArmAtBottom = false;
    private static boolean bArmContact = false;
    
    private DriverStation ds;
    int updateCounter = 0;
  //------------------------------------------------------------------------------------

    
    //Constructor added 2/04/2018 CJH modeled after the GearsBot template - getting rid of RobotMap redundancy      
    public LiftMechanism() {
		super();

		//Connect to driverstation to see if we're disabled - kill the stupid Talon warnings
		ds = DriverStation.getInstance();
		//Initial config for the Talon 
		liftTalonSRX_Master.setSensorPhase(true);  //it is out of phase by default so we need to flip it
		liftTalonSRX_Master.setSelectedSensorPosition(0, 0, 0);  //Position is zero on on the encoder when we boot up
		//We may have to set the motor inverted to have it go positive when we tell it to positive, need to check this
		//liftTalonSRX_Master.setInverted(isInverted);
        //set the speed limits
        liftTalonSRX_Master.configPeakOutputForward(LIFTER_POWER_FORWARD_LIMIT, 10);
        liftTalonSRX_Master.configPeakOutputReverse(LIFTER_POWER_REVERSE_LIMIT, 10);  //Gravity is our reverse
        //See if the Voltage compensation will give us more reliable performance
        liftTalonSRX_Master.configVoltageCompSaturation(11, 10);
        liftTalonSRX_Master.enableVoltageCompensation(true);
        
		//Set the gains - do this after testing it with the joystick
		//FWD, P, I, D, I limits that work ok for position mode (in Talon SLOT 0)
        // liftTalonSRX_Master.config_kP(0, .25, 10);  // 0.25 works if you don't limit the max setpoint difference
        liftTalonSRX_Master.config_kP(0, .8, 10);  // 0.8 works if you DO limit the max setpoint difference to ~ 500
        liftTalonSRX_Master.config_kI(0, 0, 10);
        liftTalonSRX_Master.config_kD(0, 10, 10);
        liftTalonSRX_Master.config_kF(0, 0, 10);
        
		//FWD, P, I, D, I limits that work ok for velocity mode (in Talon SLOT 1)
        liftTalonSRX_Master.config_kP(1, 2.0, 10);
        liftTalonSRX_Master.config_kI(1, 0.002, 10);
        liftTalonSRX_Master.config_kD(1, 0.0, 10);

      
		// Let's name the sensors on the LiveWindow - do this here instead of RobotMap
	    if (Robot.bDebugging) {
			//addChild("liftLimitLow", liftLimitLow);
			addChild("liftLimitHigh", liftLimitHigh);
			addChild("Talon Master",liftTalonSRX_Master);
			addChild("Talon Slave",liftTalonSRX_Slave);
	    }
	}
  //------------------------------------------------------------------------------------

    @Override
    public void initDefaultCommand() {
    	//setDefaultCommand(new LifterWithStick());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    // Stuff we don't need anymore

    @Deprecated
    //This is for controlling via the TalonSRX in velocity mode
    public void setVelocity(double vel) {
    	//Velocity we put in Slot 1, PID 0
    	liftTalonSRX_Master.selectProfileSlot(1, 0);
        liftTalonSRX_Master.set(ControlMode.Velocity,vel,1);
        liftTalonSRX_Slave.set(ControlMode.Follower, 1);
        liftTalonSRX_Master.configOpenloopRamp(0.1, 10); 
    }
    @Deprecated
    public void setVelocity(Joystick joy) {
        setVelocity(-joy.getRawAxis(OI.LIFTAXIS));
    }
    @Deprecated
    //This is for controlling via the TalonSRX in percentage output mode
    public void setOutput(double output) {
        liftTalonSRX_Master.set(ControlMode.PercentOutput, output);
        liftTalonSRX_Slave.set(ControlMode.Follower, 1);
        SmartDashboard.putNumber("power limit", LIFTER_POWER_FORWARD_LIMIT);
        SmartDashboard.putNumber("output", output);
    }
    @Deprecated
    public void setOutput(Joystick joy) {
        //modifiedOutput = -1.0*LIFTER_POWER_FORWARD_LIMIT * (joy.getRawAxis(OI.LIFTAXIS));
    	modifiedOutput = -1.0* 0.25 * (joy.getRawAxis(OI.LIFTAXIS))+ getCoastPower(getPosition());
    	setOutput(modifiedOutput);
    }
   @Deprecated
    public void stepPositionUp() {
    	double pos = Math.min(LIFTER_SETPOINT_MAX, liftTalonSRX_Master.getClosedLoopTarget(0)+ POSITION_STEP_SIZE);
        setPosition(pos);
    }
    @Deprecated
    public void stepPositionDown() {
    	double pos = Math.max(LIFTER_SETPOINT_MIN, liftTalonSRX_Master.getClosedLoopTarget(0)- POSITION_STEP_SIZE);
    	setPosition(pos);
    }
    @Deprecated
    //at the moment only allow it to step between three positions
    public void incrementPositionUp() {
    	double pos;
    	double currentSetpoint = liftTalonSRX_Master.getClosedLoopTarget(0);
    	if (currentSetpoint > LIFTER_VAULT-1) {
    		pos = LIFTER_TOP;
    	}
    	else {
    		pos = LIFTER_VAULT;
    	}
    	setPosition(pos);
    }
    @Deprecated
    //at the moment only allow it to step between three positions
    public void incrementPositionDown() {
    	double pos;
    	double currentSetpoint = liftTalonSRX_Master.getClosedLoopTarget(0);
    	if (currentSetpoint > LIFTER_VAULT +1) {
    		pos = LIFTER_VAULT;
    	}
    	else {
    		pos = LIFTER_BOTTOM;
    	}
    	setPosition(pos);
    }
    @Deprecated
    //I guess we really don't need a stop method for the arm since it's always maintaining position
    public void stop() {
        //liftTalonSRX_Master.set(ControlMode.Position, liftTalonSRX_Master.getClosedLoopTarget(0));
        //liftTalonSRX_Slave.set(ControlMode.Follower, 1);
    }
    /**
    *  Tried getting the arm to gracefully go to the top w/o a full
    *  blown motion profile - note: practice this with a cube
    *  NEED TO TURN THIS INTO A THREAD
    */
    @Deprecated
    public void lifterMoveToTop() {
    	bArmMoving = true;
    	double currentPosition = liftTalonSRX_Master.getSelectedSensorPosition(0);
    	double targetPosition=100;
    	//Twelve steps on the way up, takes a bit more than one second
    	for (int i=0; i<12; i++) {
    	  targetPosition+=500;	
    	  //Timer.delay((0.01+0.002*Math.pow(i, 1.6)));
    	  Timer.delay(0.1);
    	  if (currentPosition>4000) {
    		  targetPosition-=300;
    		  Timer.delay(0.2);
    	  }
    	  setPosition(Math.min(targetPosition,LIFTER_TOP_AUTONOMOUS));
    	}
    	bArmMoving = false;
    }
      /**
     * Straight to the switch 
     * NEED TO TURN THIS INTO A THREAD
     */
    @Deprecated
     public void lifterMoveToSwitch() {
     	bArmMoving = true;
     	double currentPosition = liftTalonSRX_Master.getSelectedSensorPosition(0);
     	double targetPosition=100;
     	//Twelve steps on the way up, takes a bit more than one second
     	for (int i=0; i<11; i++) {
     	  targetPosition+=400;	
     	  Timer.delay(0.1);
     	  setPosition(Math.min(targetPosition,LIFTER_SWITCH));
     	}
     	bArmMoving = false;
     }
    /**
    *  Tried getting the arm to gracefully go to the top w/o a full
    *  blown motion profile - note: practice this with a cube
    */
    @Deprecated
    public void lifterMoveToBottom() {
    	bArmMoving = true;
    	double currentPosition = liftTalonSRX_Master.getSelectedSensorPosition(0);
    	double targetPosition=currentPosition;
    	double kickPoint = 3500;
    	double decrement = 500;
    	if (currentPosition>5500) {
    		targetPosition = kickPoint;
        	//Kick off the top
    		setPosition(Math.max(targetPosition,100));
    	}
    	else {
    		targetPosition = currentPosition; 		
    	} 	
    	
    	//Wait until it falls past halfway
    	for (int i=60; i>0; i--) {
    		if(liftTalonSRX_Master.getSelectedSensorPosition(0)< 3000) {
    			break;
    		}
        	Timer.delay(0.05);
        }
    	Timer.delay(0.25);
    	decrement = kickPoint/10;
    	//Six steps on the way down, takes a bit more than one second
    	for (int i=10; i>0; i--) {
    	  targetPosition-=decrement;	
    	  setPosition(Math.max(targetPosition,250));
    	  Timer.delay(0.15);
    	}
    	bArmMoving = false;
    	//reset();
    }
    @Deprecated
    /**
     *   Use MoveUp and MoveDown with a button to enable it digitally
     *   Make sure that we take a slow approach to the top
     *   Slowing down 1000 units from top seems to help idiot-proof it
     */
    public void moveUpCurrent() {
    	double currentSetpoint =  Math.max(0, liftTalonSRX_Master.getClosedLoopTarget(0));
    	double currentPosition = liftTalonSRX_Master.getSelectedSensorPosition(0);
    	double maxVelocity = getMaxVelocityUp(currentPosition);
    	double velocityIncrement = 10;
    	if (currentPosition > 5500) {
    		maxVelocity = 5;
    	}
    	setVelocity(maxVelocity);
    	updateCounter++;
    	if (updateCounter++ % 5 ==0) {
    	  System.out.println("Current velocity setpoint " + String.format("%.2f",currentSetpoint) + " position " + String.format("%.2f",currentPosition));
    	}
    }
    @Deprecated
    /**
     *   Make sure that we take a slow approach to the bottom so we can't slam into it
     *   Also seems to help kick it off the top to jump the setpoint down immediately
     */
    public void moveDownCurrent() {
    	double currentSetpoint = Math.min(0, liftTalonSRX_Master.getClosedLoopTarget(0));
    	double currentPosition = liftTalonSRX_Master.getSelectedSensorPosition(0);
    	double maxVelocity = -1.0* getMaxVelocityDown(currentPosition);
    	double velocityIncrement = -10;
    	//setVelocity(Math.max(maxVelocity, currentSetpoint + velocityIncrement));
    	setVelocity(maxVelocity);
    	updateCounter++;
    	if (updateCounter++ % 5 ==0) {
    	  System.out.println("Current velocity setpoint " + String.format("%.2f",currentSetpoint) + " position " + String.format("%.2f",currentPosition));
    	}
    }
    @Deprecated
    public double getMaxVelocityUp(double pos) {
    	return Math.max(10, 9.31e-9 *pos*pos*pos -1.00e-4 *pos*pos +0.244*pos +103); 
    }
    @Deprecated
    public double getMaxVelocityDown(double pos) {
    	return Math.max(25, -1.77e-9 *pos*pos*pos +3.00e-6 *pos*pos +0.075*pos +12); 
    }

    //This is for controlling via the TalonSRX in position mode
    public void setPosition(double position) {
    	//Position we put in Slot 0, PID 0
    	liftTalonSRX_Master.selectProfileSlot(0, 0);
        liftTalonSRX_Master.set(ControlMode.Position, position);
        liftTalonSRX_Slave.set(ControlMode.Follower, 1);
    }
    public void setPosition(Joystick joy) {
        setPosition(liftTalonSRX_Master.getClosedLoopTarget(0)- JOYSTICK_POSITION_MULTIPLIER*joy.getRawAxis(OI.LIFTAXIS));
    }
        
    public void reset() {
    	liftTalonSRX_Master.setSelectedSensorPosition(0, 0, 0);  //Position is zero on on the encoder when we boot up
        liftTalonSRX_Master.set(ControlMode.Position, 0);
        liftTalonSRX_Master.set(ControlMode.Follower, 1);
        //Reset the feed forward gain
        liftTalonSRX_Master.config_kF(0, 0, 10);
    	//Position we put in Slot 0, PID 0
    	liftTalonSRX_Master.selectProfileSlot(0, 0);
        liftTalonSRX_Master.set(ControlMode.Position, 0);
        liftTalonSRX_Slave.set(ControlMode.Follower, 1);
    }
    
    public void overridePosition() {
    	liftTalonSRX_Master.setSelectedSensorPosition((int)LIFTER_ENCODER_MAX, 0, 10);  //Position is zero on on the encoder when we boot up
    	Timer.delay(0.05);
    	setPosition(LIFTER_ENCODER_MAX);
    	Timer.delay(0.05);
    	setPosition(LIFTER_ENCODER_MAX);
    	liftTalonSRX_Master.set(ControlMode.Position, 0);
        liftTalonSRX_Master.set(ControlMode.Follower, 1);
    }

	public static boolean isArmMoving() {
		return bArmMoving;
	}
	public static boolean isArmContact() {
		return bArmContact;
	}
	public static boolean isArmAtTop() {
		return bArmAtTop;
	}

	public static void setArmMoving(boolean bArmMoving) {
		LiftMechanism.bArmMoving = bArmMoving;
	}
	
    public void log() {
	  updateCounter++;
	  double liftPos = 0;
	  if (updateCounter%5 == 0) {
		  liftPos = liftTalonSRX_Master.getSelectedSensorPosition(0);
		  if (liftPos > 3000) {
			  Robot.drivetrain.setJoystickEnabled(false);
		  }
		  else {
			  Robot.drivetrain.setJoystickEnabled(true);
		  }
		  bArmAtBottom = (liftPos <200) ? true : false;
		  //bArmAtTop = (liftPos > LIFTER_ENCODER_MAX - 100) ? true : false;
		  //Want this to be false if the switch stops working, so it's wired to only be true if the switch is active  (not unplugged)
		  bArmAtTop= ! liftLimitHigh.get();
		  bArmContact = ! liftArmContact.get();
		  
		  //If we're at the top and not moving, relax the feed forward to save power
		//  if (updateCounter%10 == 0 && bArmAtTop  && !isArmMoving()) {
		//	  liftTalonSRX_Master.config_kF(0, 0.0, 10);
		//  }
		  
        SmartDashboard.putNumber("Lifter Position", liftTalonSRX_Master.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Lifter Velocity", liftTalonSRX_Master.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Lifter Power", liftTalonSRX_Master.getMotorOutputPercent());
        SmartDashboard.putNumber("Lifter Current", liftTalonSRX_Master.getOutputCurrent());
        //SmartDashboard.putBoolean("Lifter Limit Low", liftLimitLow.get());
        SmartDashboard.putBoolean("Lifter Moving", bArmMoving);
        SmartDashboard.putBoolean("Lifter At Top", bArmAtTop);
        SmartDashboard.putBoolean("Lifter At Bottom", bArmAtBottom);
        SmartDashboard.putBoolean("Arm Contact", bArmContact);
        
        if (ds.isEnabled()) {
          SmartDashboard.putNumber("Lifter Setpoint", liftTalonSRX_Master.getClosedLoopTarget(0));
        } else {
    	  SmartDashboard.putNumber("Lifter Setpoint", 0);
        }
      }
    }
    /**
     *   Worked ok - 3/29/018
     *   Use MoveUp and MoveDown with a button to enable it digitally
     *   Make sure that we take a slow approach to the top
     *   Slowing down 1000 units from top seems to help idiot-proof it
     */
    public void moveUp() {
    	double currentSetpoint = liftTalonSRX_Master.getClosedLoopTarget(0);
    	double distanceToTop = LIFTER_SETPOINT_MAX-currentSetpoint;
    	//Basically we're at full joystick speed in this method
    	double increment = JOYSTICK_RATE_UP;
    	//slow us down near the top
    	if (distanceToTop< 800) {
    		increment= Math.max(0,(distanceToTop/2000.0))* JOYSTICK_RATE_UP;
    	}
    	
    	setkF();
    	double pos = Math.min(LIFTER_SETPOINT_MAX-100, currentSetpoint+ increment);
    	double newSP = Math.min(pos, SETPOINT_ERROR_LIMIT+liftTalonSRX_Master.getSelectedSensorPosition(0));
    	
    	setPosition(newSP);
    }
    /**
     *   Worked ok - 3/29/018
     *   Make sure that we take a slow approach to the bottom so we can't slam into it
     *   Also seems to help kick it off the top to jump the setpoint down immediately
     */
    public void moveDown() {
    	double currentPosition = liftTalonSRX_Master.getSelectedSensorPosition(0);
    	double currentSetpoint = liftTalonSRX_Master.getClosedLoopTarget(0);
    	double distanceToTop = LIFTER_SETPOINT_MAX-currentSetpoint;
    	//Basically we're at full joystick speed in this method
    	double increment = JOYSTICK_RATE_DOWN;
    	double pos;
    	
    	//slow us down near the bottom
    	if (currentSetpoint< 3000) {
    		//increment= Math.max(0,(currentSetpoint/2500.0))* JOYSTICK_RATE_DOWN;
    		increment= Math.max(0.2,(currentSetpoint/3000.0))* JOYSTICK_RATE_DOWN;
    	}
    	/*
    	//kick off the top - this is the best way I've found to come down quickly
    	if (currentPosition>5500) {
    		pos = 4000;
    	}
    	else {
    		pos = Math.max(LIFTER_SETPOINT_MIN, currentSetpoint - increment);
    	}
    	*/
    	//Maybe this is a bit smarter for slowing us down as we go lower - give us 25 to 100% of the max step
    	//double scaledHeight = Math.max(0.25, currentPosition/LIFTER_SETPOINT_MAX);
    	//increment = scaledHeight*JOYSTICK_RATE_DOWN;
    	setkFDown();
    	pos = Math.max(LIFTER_SETPOINT_MIN, currentSetpoint - increment);
    	double newSP = Math.max(pos, - SETPOINT_ERROR_LIMIT+liftTalonSRX_Master.getSelectedSensorPosition(0));
    	setPosition(pos);
    }


    public void holdPosition() {
    	double pos = liftTalonSRX_Master.getSelectedSensorPosition(0);
    	setPosition(pos);
    	System.out.println("Holding arm at " + String.format("%.2f",pos) + "s");
    	
    }
    
    public double getPosition() {
    	return liftTalonSRX_Master.getSelectedSensorPosition(0); 
    }
    public double getArmSetpoint() {
    	return liftTalonSRX_Master.getClosedLoopTarget(0);
    }
    public double getPower() {
    	return liftTalonSRX_Master.getMotorOutputPercent(); 
    }

    public double getCoastPower(double pos) {
    	//Smooth curve from 0.3 at bottom to -0.2 at top
    		return -0.28639198798713633 + 0.3970999659246556/(1 + 0.8508286003148655/Math.pow(Math.E,0.0007647286847900521*(2420.644985297508 - pos))) + 0.23198819989034095/(1 + 1.3924357794737954/Math.pow(Math.E,0.008122602098620034*(4963.200211629625 - pos)));
    }
    
    public void setkF(double kf) {
    	liftTalonSRX_Master.config_kF(0, kf, 10);
    }
    public void setkF() {
    	double pos = liftTalonSRX_Master.getSelectedSensorPosition(0);
    	double kf = 0.03 + -0.07221198669833977 + 0.6333760179814649/(1 + 2.6357627520362685/Math.pow(Math.E,0.0005915086624398398*(572.668518403214 - pos))) + 0.05367424163613976/(1 + 0.8379848517362044/Math.pow(Math.E,0.006516894786510444*(4928.427319457754 - pos)));
    	liftTalonSRX_Master.config_kF(0, kf, 10);
    }
    //Haven't implemented this yet, but we should have a different up and down kf to make it faster
    public void setkFUp() {
    	double pos = liftTalonSRX_Master.getSelectedSensorPosition(0);
    	double kf = 0.03 + -0.07221198669833977 + 0.6333760179814649/(1 + 2.6357627520362685/Math.pow(Math.E,0.0005915086624398398*(572.668518403214 - pos))) + 0.05367424163613976/(1 + 0.8379848517362044/Math.pow(Math.E,0.006516894786510444*(4928.427319457754 - pos)));
    	liftTalonSRX_Master.config_kF(0, kf, 10);
    }
    public void setkFDown() {
    	double pos = liftTalonSRX_Master.getSelectedSensorPosition(0);
    	double kf = -0.03 + -0.07221198669833977 + 0.6333760179814649/(1 + 2.6357627520362685/Math.pow(Math.E,0.0005915086624398398*(572.668518403214 - pos))) + 0.05367424163613976/(1 + 0.8379848517362044/Math.pow(Math.E,0.006516894786510444*(4928.427319457754 - pos)));
    	liftTalonSRX_Master.config_kF(0, kf, 10);
    }
    public void overrideLowerSetpoint(double min) {
    	LIFTER_SETPOINT_MIN = min;	
    }
}