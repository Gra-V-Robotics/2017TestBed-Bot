package org.usfirst.frc.team5816.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	Talon frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
	
	RobotDrive driveTrain = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
	
	Joystick logitechController;

	@Override
	public void robotInit() {
		frontLeftMotor = new Talon(0);
		frontRightMotor = new Talon(1);
		backLeftMotor = new Talon(2);
		backRightMotor = new Talon(3);
		
		logitechController = new Joystick(0);
	}

	@Override
	public void autonomousInit() {

	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopPeriodic() {
		
		driving(logitechController.getRawAxis(1), logitechController.getRawAxis(2));
		
	}

	@Override
	public void testPeriodic() {
	}
	
	public void driving(double leftValue, double rightValue) {
		driveTrain.tankDrive(leftValue, rightValue);
	}
	
}