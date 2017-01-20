package org.usfirst.frc.team5816.robot;

import java.util.Properties;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	//Talon frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
	
	CANTalon frontLeftMotor = new CANTalon(4);
	CANTalon frontRightMotor = new CANTalon(2);
	CANTalon backLeftMotor = new CANTalon(5);
	CANTalon backRightMotor = new CANTalon(3);
//	CANTalon test = new CANTalon(4);
	Joystick logitechController = new Joystick(0);
	
//	RobotDrive driveTrain = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

	@Override
	public void robotInit() {

	}

	@Override
	public void autonomousInit() {

	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopPeriodic() {
		System.out.println(logitechController.getRawAxis(1));
		frontLeftMotor.set(logitechController.getRawAxis(1));
		backLeftMotor.set(logitechController.getRawAxis(1) * -1);
		frontRightMotor.set(logitechController.getRawAxis(5) * -1);
		backRightMotor.set(logitechController.getRawAxis(5) * -1);
		//driving(logitechController.getRawAxis(1), logitechController.getRawAxis(2));
		
	}

	@Override
	public void testPeriodic() {
	}
	
//	public void driving(double leftValue, double rightValue) {
//		driveTrain.tankDrive(leftValue, rightValue);
//	}
	
}