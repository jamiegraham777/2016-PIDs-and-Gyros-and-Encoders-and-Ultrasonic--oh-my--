
package org.usfirst.frc.team4043.robot;

import org.usfirst.frc.team4043.robot.AnalogUltrasonic.DistanceUnits;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;

//******************[ Start Code ]*********************************************************************

public class Robot extends IterativeRobot {

	public Joystick xbox, xbox2; // xbox; // set to ID 1 in DriverStation

	public Timer timer = new Timer();

	public Talon armMotor1 = new Talon(Config.armSimMotor);
	public Talon armMotor2 = new Talon(Config.armWindowMotor);
	
	public SingleEncoder leftEncoder = new SingleEncoder(Config.leftEncoder, false, Encoder.EncodingType.k1X);
	public SingleEncoder rightEncoder = new SingleEncoder(Config.rightEncoder, false, Encoder.EncodingType.k1X);
	public SingleEncoder armEncoder = new SingleEncoder(Config.armEncoder, false, Encoder.EncodingType.k1X);
	
	int autocounter;
	int count, rawCount;
	double distance, period, rate, VPI = 0.00935547;
	boolean direction, stopped, armCompleted;
	double offset;	
	
	PIDController driveControl;
	
	public AnalogUltrasonic sonic = new AnalogUltrasonic(2, DistanceUnits.INCHES, VPI);
	public AnalogGyro gyro = new AnalogGyro(1);
	//LaserCounter laserCounter;
	Drive drive;
	
	
	
	

	public void RobotInit() {
		
		

		leftEncoder.setDistancePerPulse(12 * Math.PI * ((1 / 1024f) / 2)); // 12 inches times pi divided by gearbox ratio (2.5 to 5) divided by fps
		rightEncoder.setDistancePerPulse(12 * Math.PI * ((1 / 1024f) / 2));
		armEncoder.setDistancePerPulse(12 * Math.PI * ((1 / 1024f) / 2));
		
		leftEncoder.setSamplesToAverage(7);
		rightEncoder.setSamplesToAverage(7);
		armEncoder.setSamplesToAverage(7);
		
		leftEncoder.reset();
		rightEncoder.reset();
		armEncoder.reset();
		
		armCompleted = true;
//		laserCounter = new LaserCounter();
		
		gyro.calibrate();
    	gyro.initGyro();
    	
    	xbox = new Joystick(0);
    	xbox2 = new Joystick(1);
    	timer.start();

		sonic = new AnalogUltrasonic(2, DistanceUnits.INCHES, VPI);
		sonic.setAverageBits(3);  //(note it is 2^2)
		
		drive = new Drive(xbox, xbox2, gyro);
	}

	public void operatorControl() {

	

		 
		while (isOperatorControl() && isEnabled()) {
			
//			
//			System.out.println("Distance:  " + sonic.getDistance());
//			System.out.println("Volt: " + sonic.getVoltage());
//			System.out.println("AvgDistance " + sonic.getAverageDistance());
//			System.out.println("Gyro: " +  gyro.getAngle() * 0.03);
		//	System.out.println("offset: " + offset);
		//	System.out.println("value " + ((sonic.getValue() - offset1) / VPI));
			
			Timer.delay(0.1);
			
			//System.out.println("Gryo: " + (gyro.getAngle() * 0.03));

			if (xbox.getRawButton(2) == true) {
				armCompleted = false;
	//			laserCounter.reset();
			}
			//driveArm(); 

			if (xbox.getRawAxis(2) > 0.5) { // bring arm 1 down --- button B
				/* if (leftEncoder.getDistance() < 45) */
				armMotor2.set(0.15);
			} else if (xbox.getRawAxis(2) < 0.5 && xbox.getRawAxis(3) < 0.5) {
				armMotor2.set(0);
			}
			if (xbox.getRawAxis(3) > 0.5) { // bring arm 1 up --- button A
				/* if (leftEncoder.getDistance() < 45) */
				armMotor2.set(-0.15);
			} else if (xbox.getRawAxis(2) < 0.5 && xbox.getRawAxis(3) < 0.5) {
				armMotor2.set(0);
			}
			
			if (xbox.getRawAxis(1) > 0.1) { // bring arm 1 down --- button B
				/* if (leftEncoder.getDistance() < 45) */
				armMotor1.set(0.3);
			} else if (xbox.getRawAxis(1) < 0.1 && xbox.getRawAxis(1) > -0.1) {
				armMotor1.set(0);
			}
			if (xbox.getRawAxis(1) < -0.1) { // bring arm 1 up --- button A
				/* if (leftEncoder.getDistance() < 45) */
				armMotor1.set(-0.3);
			} else if (xbox.getRawAxis(1) < 0.1 && xbox.getRawAxis(1) > -0.1) {
				armMotor1.set(0);
			}
			System.out.println(xbox.getRawAxis(1));
			
		//	System.out.println("armEncoder : " + armEncoder.getRaw());
			// System.out.println("left: " + leftEncoder.getRaw());
			// System.out.println("Normal: " + NormalEncoder.getRaw());
			// System.out.println("right " + rightEncoder.getRaw());

			// ****************[ drives robot
			// ]*********************************************************************

//			if (xbox.getRawAxis(5) > 0.15 || xbox.getRawAxis(5) < -0.15) {
//				if ((xbox.getRawAxis(5) > 0.5 && xbox.getRawAxis(1) > 0.5)
//						|| ((xbox.getRawAxis(5) < -0.5 && xbox.getRawAxis(1) < -0.5))) {
//					leftEncoder.reset();
//					// rightEncoder.reset();
//					if ((xbox.getRawAxis(5) > 0.75 && xbox.getRawAxis(1) > 0.75)
//							|| ((xbox.getRawAxis(5) < -0.75 && xbox.getRawAxis(1) < -0.75))) {
//						driveStraight(xbox.getRawAxis(5));
//					}
//				} else if ((xbox.getRawAxis(5) > 0.15 && xbox.getRawAxis(5) < 0.75)
//						|| (xbox.getRawAxis(5) < -0.15 && xbox.getRawAxis(5) > -0.75)) {
//					// System.out.println("left: " + leftEncoder.getRaw());
//					// System.out.println("Normal: " + rightEncoder.getRaw());
//					FrightWheel.set(-xbox.getRawAxis(5)); // * (1-(offset
//															// *0.1)));
//					BrightWheel.set(-xbox.getRawAxis(5));
//				}
//			} else if (xbox.getRawAxis(5) < 0.15 || xbox.getRawAxis(5) > -0.15) {
//				// leftEncoder.reset();
//				// NormalEncoder.reset();
//				FrightWheel.set(0);
//				BrightWheel.set(0);
//			} else if (xbox.getRawAxis(1) > 0.15 || xbox.getRawAxis(1) < -0.15) {
//				System.out.println("left: " + leftEncoder.getRaw());
//				System.out.println("Normal: " + rightEncoder.getRaw());
//				FleftWheel.set(xbox.getRawAxis(1));
//				BleftWheel.set(xbox.getRawAxis(1));
//			} else if (xbox.getRawAxis(1) < 0.15 || xbox.getRawAxis(1) > -0.15) {
//				// leftEncoder.reset();
//				// NormalEncoder.reset();
//				FleftWheel.set(0);
//				BleftWheel.set(0);
//			}

		}

		Timer.delay(0.005); //Needs to be 0.005 for competition
	}

	private void driveArm() {

//		if (armCompleted == false) {
//			System.out.println("ArmEncoder: " + armEncoder.getRaw());
//			if (armEncoder.getRaw() < 450) {
//				armMotor1.set(0.15);
//			} else if (armEncoder.getRaw() < 900) {
//				armMotor1.set(-0.1);
//			} else if (armEncoder.getRaw() > 900) {
//				armCompleted = true;
//				armMotor1.set(0);
//				armEncoder.reset();
//			}
//		}
		
//		if (armCompleted == false) {
//			int count = laserCounter.laserCheck();
//			if (count < 10) {
//				armMotor2.set(0.75);
//				System.out.println("One " + count);
//			} else if (count < 20) {
//				armMotor2.set(-0.75);
//				System.out.println("Two " + count);
//			} else if (count > 20) {
//				armCompleted = true;
//				armMotor2.set(0);
//				laserCounter.reset();
//				System.out.println("Three");
//			}
//		}
	}

	public void driveStraight(double num) {
		System.out.println("left: " + leftEncoder.getRaw());
		System.out.println("Normal: " + rightEncoder.getRaw());
		System.out.println("offset: " + offset);
		// System.out.println("right" + rightEncoder.getRaw());
		double adder = 0;
		int negornot = 1;
		double offset = 0;
		offset = (leftEncoder.getRaw() - rightEncoder.getRaw());
		if (num < 0)
			negornot *= -1;

		if (offset > 0) {
			System.out.println("left needs to be lowered");
	//		FrightWheel.set(negornot * (1));
			System.out.println("right wheel: " + (negornot * (1)));
			adder = Math.abs(offset);
			if (adder > 10) {
				adder = adder / 10;
			}
			System.out.println("adder: " + adder);
			System.out.println("left wheel: " + (negornot * (1 - (adder * 0.1))));
	//		FleftWheel.set(negornot * (1 - (adder * 0.1)));
		} else if (offset < 0) {
			System.out.println("right needs to be lowered");
	//		FleftWheel.set(negornot * (1));
			System.out.println("left wheel: " + (negornot * (1)));
			adder = Math.abs(offset);
			while (adder > 10) {
				adder = adder / 10;
			}
			System.out.println("right wheel: " + (negornot * (1 - (adder * 0.1))));
		//	FrightWheel.set(negornot * (1 - (adder * 0.1)));
		}

	}

	public void autonomous() {

		if (isEnabled()) {

			/*
			 * leftWheel.set(0.5); // backs up to victory (auto zone)
			 * rightWheel.set(-0.5);
			 * 
			 * Timer.delay(2.7); //depends on battery count
			 * 
			 * leftWheel.set(0); rightWheel.set(0); Timer.delay(0.005);
			 * 
			 * 
			 * Timer.delay(12.3); //Just in case it loops
			 */
		}
	}
}