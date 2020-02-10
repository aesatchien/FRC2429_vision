package org.usfirst.frc.team2429.spartan2017.buttons;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class PovButtonDown extends Button{
	Joystick joystick;
	int axis;
	
	public PovButtonDown(Joystick joystick){
		this.joystick = joystick;
		
	}
	public boolean get(){
		//Down is 180.  So if it's not 180, the button isn't pressed  - CJH
		//Note, nobody being pressed is -1
		return joystick.getPOV() == 180;
	}
}
