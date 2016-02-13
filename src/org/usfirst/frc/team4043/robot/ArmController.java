package org.usfirst.frc.team4043.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;

public class ArmController {

	static Talon armMotor1 = new Talon(Config.armSimMotor);
	static Talon armMotor2 = new Talon(Config.armSimMotor);

	private LaserCounter laserCounter = new LaserCounter();
	private SingleEncoder armEncoder = new SingleEncoder(Config.armEncoder, false, Encoder.EncodingType.k1X);
	
	public final static int FORWARD = 1;
	public final static int BACKWARD = -1;
	
	private boolean isRunning;
	private Step[] currentProgram;
	private int whichStep;
	
	public final Step[] TestLowerArmSteps = new Step[] {
			new Step(armMotor1, 10, FORWARD),
			new Step(armMotor1, 10, BACKWARD)
	};
	
	public void run() {
		if (isRunning == false){
			return;
		}
	}
	
	private boolean arm1Done (int number) {
		if (laserCounter.counter >= number) {
			return true;
		}else {
			return false;
		}
	}
	
	private boolean arm2Done (int number) {
		if (armEncoder.getRaw() >= number) {
			return true;
		}else {
			return false;
		}
	}
	
	private class Step {
		Talon motor;
		int ticks;
		int direction;
		
		Step (Talon m, int t, int d) {
			motor = m;
			ticks = t;
			direction = d;
		}
	}
}
