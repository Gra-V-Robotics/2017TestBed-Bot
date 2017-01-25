package org.usfirst.frc.team5816.robot;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.Properties;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	
	Encoder leftEncoder = new Encoder(0, 1, true, EncodingType.k4X);
	Encoder rightEncoder = new Encoder(2, 3, true, EncodingType.k4X);
	
	Gyro gyro = new ADXRS450_Gyro();
	
	Joystick logitechController = new Joystick(0);
	
	NetworkTable table;
	
	CANTalon frontLeftMotor = new CANTalon(4);
	CANTalon frontRightMotor = new CANTalon(2);
	CANTalon backLeftMotor = new CANTalon(5);
	CANTalon backRightMotor = new CANTalon(3);
	
	double gyroAngleVal;
	double gyroRateVal;
	double leftEncoderVal;
	double rightEncoderVal;
	
	RobotDrive driveTrain = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

	@Override
	public void robotInit() {
		sensorCalibration();
		//this.table = NetworkTable.getTable("ShooterCamera");
	}

	@Override
	public void autonomousInit() {
		resetSensors();
	}

	@Override
	public void autonomousPeriodic() {

	}
	
	@Override
	public void teleopInit() {
		resetSensors();
		
	}

	@Override
	public void teleopPeriodic() {
		dynamicVariableInitilization();
		driveWithJoysticks(logitechController.getRawAxis(1), logitechController.getRawAxis(5));
		SmartDashboard.putNumber("GyroAngleVal: ", gyroAngleVal);
		SmartDashboard.putNumber("GyroRateVal: ", gyroRateVal);
		SmartDashboard.putNumber("LeftEncoderVal: ", leftEncoderVal);
		SmartDashboard.putNumber("RightEncoderVal: ", -rightEncoderVal);
	}

	@Override
	public void testPeriodic() {
		
	}
	
	public void resetSensors() {
		leftEncoder.reset();
		rightEncoder.reset();
		
		gyro.reset();
	}
	
	public void sensorCalibration() {
		gyro.calibrate();
	}
	
	public void driveWithJoysticks(double leftVal, double rightVal) {
		driveTrain.tankDrive(-leftVal, -rightVal);
	}
	
	public void dynamicVariableInitilization() {
		gyroAngleVal = gyro.getAngle();
		gyroRateVal = gyro.getRate();
		leftEncoderVal = leftEncoder.get();
		rightEncoderVal = rightEncoder.get();
	}

}