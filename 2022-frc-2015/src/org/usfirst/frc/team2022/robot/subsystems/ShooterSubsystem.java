package org.usfirst.frc.team2022.robot.subsystems;

import org.usfirst.frc.team2022.robot.commands.ShooterCommand;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ShooterSubsystem extends Subsystem {
    
    private final Talon rightMotor, leftMotor;
    
    public ShooterSubsystem(){
    	rightMotor = new Talon(5);
    	leftMotor = new Talon(4);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new ShooterCommand());
    }
    
    public void setRightSpeed(double speed){
    	rightMotor.set(speed);
    }
    
    public void setLeftSpeed(double speed){
    	leftMotor.set(speed);
    }
    
    public void stop(){
    	leftMotor.set(0);
    	rightMotor.set(0);
    }
}

