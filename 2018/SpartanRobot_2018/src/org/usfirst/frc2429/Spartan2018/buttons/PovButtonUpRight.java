package org.usfirst.frc2429.Spartan2018.buttons;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class PovButtonUpRight extends Button{
	Joystick joystick;
	int axis;
	
	public PovButtonUpRight(Joystick joystick){
		this.joystick = joystick;
		
	}
	public boolean get(){
		//Right is 90.  So if it's not 90, the button isn't pressed  - CJH
		//Note, nobody being pressed is -1
		return joystick.getPOV() == 45;
	}
}
