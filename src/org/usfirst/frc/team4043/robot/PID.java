package org.usfirst.frc.team4043.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class PID implements Updatable {
	
	//volatile = variables that may be accessed from a separate thread
	
	public volatile double maxErrordt, maxError, maxI, setPoint;
	public volatile double kP, kI, kD;
	public volatile boolean onTarget = false, enabled = false;
//	private double errorSum;
//	private double error;
	private double i, previousError;
//	private double previousTime;
//	private double output;
//	private double errorMax = 0;
//	private double errorMin = 0;	
//	private Timer timer;
	
	private PIDSource source;
	private PIDOutput output;
	
//	public PID(double p, double i, double d) {
//		kP = p;
//		kI = i;
//		kD = d;
//		previousTime = 0;
//		output = 0;
//	}
	public PID(PIDSource source, PIDOutput output) {
		this.source = source;
		this.output = output;
	}
	
	public void update(double current, double want) {
		double dError;
		double dt = timer.get() - previousTime;   // Delta T
		previousTime = timer.get();
		error = current - want;
		errorSum += error * dt;
		
		if (errorSum > errorMax) {
			errorSum = errorMax;
		}
		else if (errorSum < errorMin) {
			errorSum = errorMin;
		}
	
		dError = (error - previousError) / dt;
		output = (error * kP) + (errorSum * kI) + (dError * kD);
		previousError = error;
		
	}
	
	public void reset() {
		output = 0;
		errorSum = 0;
		previousError = 0;
		previousTime = 0;
	}

	public double getOutput() {
		return output;
	}

	public void setErrorThreshold(double min, double max) {
		errorMax = max;
		errorMin = min;
	}
}
