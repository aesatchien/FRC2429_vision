package org.usfirst.frc.team2429.spartan2017.buttons;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class PovButtonUp extends Button{
	Joystick joystick;
	int axis;
	
	public PovButtonUp(Joystick joystick){
		this.joystick = joystick;
		
	}
	public boolean get(){
		//Up is 0.  So if it's not zero, the button isn't pressed  - CJH
		//Note, nobody being pressed is -1
		return joystick.getPOV() == 0;
	}
}
