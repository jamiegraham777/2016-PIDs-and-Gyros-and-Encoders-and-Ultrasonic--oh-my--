package org.usfirst.frc.team4043.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class LaserCounter {
	int counter = 0;
	boolean isHigh = false;
	double reading = 0;
	AnalogInput armlaser;
	double limit = 1.5;
	
	public LaserCounter() {
		armlaser = new AnalogInput(Config.armIRport);
		
	}
	
	public int laserCheck () {
		reading = armlaser.getVoltage();
		//System.out.println("Reading " + reading);
		
		if (!isHigh && reading > limit) {
			isHigh = true;
			counter ++;
		}else if (isHigh && reading < limit) {
			isHigh = false;
		}
		
		return counter;
	}
	
	public void reset () {
		counter = 0;
	}
}
