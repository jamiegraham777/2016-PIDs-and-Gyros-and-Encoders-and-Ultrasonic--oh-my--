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
	
	public static class PIDConstants {
		public double kP, kI, kD;
		
		public PIDConstants(double _kP, double _kI, double _kD) {
			kP = _kP;
			kI = _kI;
			kD = _kD;
		}
	}
	public PID(PIDSource source, PIDOutput output) {
		this.source = source;
		this.output = output;
	}
	
	public synchronized void setGains(PIDConstants constants) {
		kP = constants.kP;
		kI = constants.kI;
		kD = constants.kD;
	}
	
	public void update() {
		
		double dt = 0.02;
		double error = setPoint - source.pidGet();
		double p = error;
		
		i += error * dt;
		
		coerce(i, -maxI, maxI);
		
		double errorDt = (error - previousError) / dt;
		double d = errorDt;
		
		double result = kP * p + kI * i + kD * d;
		
		if ((Math.abs(errorDt) < maxErrordt) && (Math.abs(error) < maxError)) {
			i =  onTarget ? 0 : i;
		}
		
		if (enabled) {
			output.pidWrite(result);
		}
		
		previousError = error;
	}
		public static double coerce(double a, double min, double max) {
			return Math.min(max,  Math.max(a, min));
		}
		
		public boolean isOld() {
		return false;	
		}
//		double dError;
//		double dt = timer.get() - previousTime;   // Delta T
//		previousTime = timer.get();
//		error = current - want;
//		errorSum += error * dt;
//		
//		if (errorSum > errorMax) {
//			errorSum = errorMax;
//		}
//		else if (errorSum < errorMin) {
//			errorSum = errorMin;
//		}
//	
//		dError = (error - previousError) / dt;
//		output = (error * kP) + (errorSum * kI) + (dError * kD);
//		previousError = error;
		
//	}
//	
//	public void reset() {
//		output = 0;
//		errorSum = 0;
//		previousError = 0;
//		previousTime = 0;
//	}
//
//	public double getOutput() {
//		return output;
//	}
//
//	public void setErrorThreshold(double min, double max) {
//		errorMax = max;
//		errorMin = min;
//	} 
}
