package org.usfirst.frc.team4043.robot;


import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;

public class Drive {
	private CANTalon Bleft;
	private CANTalon Bright;
	private CANTalon Fright;
	private CANTalon Fleft;
	private Joystick jox, jox2;
	private AnalogGyro gyro;
	private	PID clockwisePID;
	private double max;
	
	public Drive(Joystick jox1, Joystick jox2, AnalogGyro gyro) {
		this.jox2 = jox2;
		this.jox = jox1;
		this.gyro = gyro;
		
//		clockwisePID = new PID(Config.clockwiseP, Config.clockwiseI, Config.clockwiseD);
		
		Bleft = new CANTalon(Config.BleftWheel);
		Bright = new CANTalon(Config.BrightWheel);
		Fright = new CANTalon(Config.FrightWheel);
		Fleft = new CANTalon(Config.FleftWheel);
	}
	
	public void run(){
		double forward = -jox.getRawAxis(1); // push joystick1 forward to go forward
		double right = jox.getRawAxis(0); // push joystick1 to the right to strafe right
		double clockwise = jox.getRawAxis(2); // push joystick2 to the right to rotate clockwise 
		double want = Math.atan2(jox.getRawAxis(1), jox.getRawAxis(0));
		double have = gyro.getAngle();
		
	//	clockwisePID.update(have,want);
		
	//	fieldCentricSetSpeed(forward, right, clockwise, gyro.getAngle(), clockwisePID.getOutput());
	}
	
	private void fieldCentricSetSpeed(double forward, double right, double clockwise, double theta, double PIDCorrection) {
		theta = theta*180/Math.PI;
		double temp = forward*Math.cos(theta) + right*Math.sin(theta);
		right = -forward*Math.sin(theta) + right*Math.cos(theta);
		forward = temp; 
		
		setSpeed(forward, right, clockwise + PIDCorrection);
	}
	
	//max = Math.max(Math.max(Math.abs(rear_left), Math.abs(rear_right)), Math.max(Math.abs(front_left), Math.abs(front_left)));
	//max = Math.max(Math.max(rear_left, rear_right), Math.max(front_left, front_left));
	public void setSpeed(double forward, double right, double clockwise) {
		double front_right= Fright.get();
		double front_left = Fleft.get();
		double back_left = Bleft.get();
		double back_right = Bright.get();
		
		max = Math.abs(front_left);
		max = Math.max(Math.abs(front_right), max);
		max = Math.max(Math.abs(back_left), max);
		max = Math.max(Math.abs(back_right), max);
		
		if (max > 1) {
			front_left  /= max; 
			front_right /= max; 
			back_left   /= max; 
			back_right  /= max;
		} 
		
		Fleft.set(forward + clockwise + right);
		Fright.set(forward - clockwise - right);
		Bleft.set(forward + clockwise - right);
		Bright.set(forward - clockwise + right); 
	}

}
