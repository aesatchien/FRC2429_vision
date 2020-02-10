package org.usfirst.frc.team2429.spartan2017.buttons;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class PovButtonRight extends Button{
	Joystick joystick;
	int axis;
	
	public PovButtonRight(Joystick joystick){
		this.joystick = joystick;
		
	}
	public boolean get(){
		//Right is 90.  So if it's not 90, the button isn't pressed  - CJH
		//Note, nobody being pressed is -1
		return joystick.getPOV() == 90;
	}
}
