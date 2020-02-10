package org.usfirst.frc.team2429.spartan2017.buttons;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class AxisButton extends Button{
	Joystick joystick;
	int axis;
	
	public AxisButton(Joystick joystick, int axis){
		this.joystick = joystick;
		this.axis = axis;
		
	}
	public boolean get(){
		return joystick.getRawAxis(axis) > 0;
	}
}
