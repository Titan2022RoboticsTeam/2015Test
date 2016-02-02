package org.usfirst.frc.team2022.robot.subsystems;

import org.usfirst.frc.team2022.robot.RobotMap;
import org.usfirst.frc.team2022.robot.commands.TankDriveCommand;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TankDriveSubsystem extends Subsystem {
	
	private TalonSRX frontLeft, frontRight, rearLeft, rearRight;
	private double leftSpeed, rightSpeed;
	private boolean inverted;
	private long lastTime;
	
    
	//Encoders
	public static Encoder rightEncoder;
	public static Encoder leftEncoder;
	//Encoder Constants
	public static final double driveWheelRadius = 2.5;
	public static final int pulsePerRotation = 360;
	public static final double gearRatio = 1/1;
	public static final double driveEncoderPulsePerRot = pulsePerRotation * gearRatio;
	public static final double driveEncoderDistPerTick = (Math.PI * driveWheelRadius * 2) / driveEncoderPulsePerRot;
	public static final double driveEncoderMaxPeriod = .1;
	public static final double driveEncoderMinRate = 10;
	
	public static final double rightP = 1;
	public static final double rightI = 0.1;
	public static final double rightD = 0.1;
	public static final double rightK = 0.1;

	
	public static final double leftP = 1;
	public static final double leftI = 0.1;
	public static final double leftD = 0.1;
	public static final double leftK = 0.1;

	
	public PIDController rightController;
	public PIDOutputRight pidOutputRight;
	
	public PIDController leftController;
	public PIDOutputLeft pidOutputLeft;
	
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public TankDriveSubsystem(){
		frontLeft = new TalonSRX(RobotMap.leftMotorFront);
		frontRight = new TalonSRX(RobotMap.rightMotorFront);
		rearLeft = new TalonSRX(RobotMap.leftMotorBack);
		rearRight = new TalonSRX(RobotMap.rightMotorBack);
		
		//Initialize Encoders
		rightEncoder = new Encoder(RobotMap.rightEncoderPortA, RobotMap.rightEncoderPortB, false, CounterBase.EncodingType.k4X);
		leftEncoder = new Encoder(RobotMap.leftEncoderPortA, RobotMap.leftEncoderPortB, false, CounterBase.EncodingType.k1X);
		//Set Encoder distance per pulse
		rightEncoder.setDistancePerPulse(driveEncoderDistPerTick);
		leftEncoder.setDistancePerPulse(driveEncoderDistPerTick);	
		
		
//		pidOutputRight = new PIDOutputRight();
//		rightController = new PIDController(rightP, rightI, rightD, rightK, rightEncoder, pidOutputRight);
//
//		pidOutputLeft = new PIDOutputLeft();
//		leftController = new PIDController(leftP, leftI, leftD, leftK, leftEncoder, pidOutputLeft);
		
//		rightController.setOutputRange(-1, 1);
//		leftController.setOutputRange(-1, 1);
		
		
		inverted = false;
		lastTime = System.currentTimeMillis();
    
	}

    public void initDefaultCommand() {
    	setDefaultCommand(new TankDriveCommand());

    }	
    
 // Speed Manipulation Methods-these are more fine grained
 	public double getLeftSpeed() {
 		return leftSpeed;
 	}

 	public double getRightSpeed() {
 		return rightSpeed;
 	}

 	public void setLeftSpeed(double ls) {
 		leftSpeed = ls;
 		frontLeft.set(ls);
 		rearLeft.set(ls);
 	}

 	public void setRightSpeed(double rs) {
 		rightSpeed = rs;
 		frontRight.set(rs);
 		rearRight.set(rs);
 	}

 	// Inversion
 	public boolean isInverted() {
 		return inverted;
 	}

 	public void toggleInversion() {
 		if (System.currentTimeMillis() > lastTime + 250) {
 			lastTime = System.currentTimeMillis();
 			inverted = !inverted;
 			leftSpeed *= -1;
 			rightSpeed *= -1;
 		}
 	}

	//Get Encoder Distances
	public double getRightEncoderDistance(){
		System.out.println(rightEncoder.getDistance());
		return rightEncoder.getDistance();
	}
	
	public double getLeftEncoderDistance(){
		System.out.println(leftEncoder.getDistance());
		return leftEncoder.getDistance();
	}
	
	public double getLeftEncoderRawValue(){
		System.out.println(leftEncoder.get());
		return leftEncoder.get();
	}
	
	public double getRightEncoderRawValue(){
		System.out.println(rightEncoder.get());
		return rightEncoder.get();
	}
	//Get Encoder Rates
	public double getRightEncoderRate(){
		return rightEncoder.getRate();
	}
	
	public double getLeftEncoderRate(){
		return leftEncoder.getRate();
	}
	
//	public void resetEncoders(){
//		rightEncoder.reset();
//		leftEncoder.reset();
//	}

	// Forwards and Reverse Control for each side.
	public void stop() {
		frontRight.set(0);
		frontLeft.set(0);
		rearRight.set(0);
		rearLeft.set(0);
		rightSpeed = 0;
		leftSpeed = 0;
	}

	public void driveDistanceStraight(double inches) {
//		rightController.setSetpoint(inches);
//		leftController.setSetpoint(inches);
//		rightController.enable();
//		leftController.enable();
		while(true){
//			setRightSpeed(pidOutputRight.getOutput());
//			setLeftSpeed(pidOutputLeft.getOutput());
			SmartDashboard.putNumber("Right Speed", getRightEncoderRate());
			SmartDashboard.putNumber("Right Encoder Distance", getRightEncoderDistance());
			SmartDashboard.putNumber("Right P", rightP);
			SmartDashboard.putNumber("Right I", rightI);
			SmartDashboard.putNumber("Right D", rightD);
			SmartDashboard.putNumber("Right K", rightK);
			SmartDashboard.putNumber("Left Speed", getLeftEncoderRate());
			SmartDashboard.putNumber("Left Encoder Distance", getLeftEncoderDistance());
			SmartDashboard.putNumber("Left P", rightP);
			SmartDashboard.putNumber("Left I", rightI);
			SmartDashboard.putNumber("Left D", rightD);
			SmartDashboard.putNumber("Left K", rightK);
		}

	}
}

