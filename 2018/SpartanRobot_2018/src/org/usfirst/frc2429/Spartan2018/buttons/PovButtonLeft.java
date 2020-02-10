package org.usfirst.frc2429.Spartan2018.buttons;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

public class PovButtonLeft extends Button{
	Joystick joystick;
	int axis;
	
	public PovButtonLeft(Joystick joystick){
		this.joystick = joystick;
		
	}
	public boolean get(){
		//Left is 270.  So if it's not 270, the button isn't pressed  - CJH
		//Note, nobody being pressed is -1
		return joystick.getPOV() == 270;
	}
}
