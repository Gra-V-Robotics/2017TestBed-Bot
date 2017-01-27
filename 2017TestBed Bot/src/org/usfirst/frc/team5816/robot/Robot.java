package org.usfirst.frc.team5816.robot;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.Properties;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PIDCommand;
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
	
	//AHRS navX = new AHRS(SerialPort.Port.kMXP);
	
	CANTalon frontLeftMotor = new CANTalon(4);
	CANTalon frontRightMotor = new CANTalon(2);
	CANTalon backLeftMotor = new CANTalon(5);
	CANTalon backRightMotor = new CANTalon(3);
	
	double gyroAngleVal;
	double gyroRateVal;
	double leftEncoderVal;
	double rightEncoderVal;
	
	RobotDrive driveTrain = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
	
	PIDController anglePIDController;
	PIDSource anglePIDSource;
	PIDOutput anglePIDOutput;
	
	Double angleKp;
	Double angleKi;
	Double angleKd;
	
	Double angleOutput;
	
	PIDController encoderPIDController;
	PIDSource encoderPIDSource;
	PIDOutput encoderPIDOutput;
	
	Double encoderKp;
	Double encoderKi;
	Double encoderKd;
	
	Double encoderOutput;
	
	DriverStation driverStation; 
	
	Double battery;
	Alliance teamAlliance;
	Double matchTime;

	@Override
	public void robotInit() {
		sensorCalibration();
		gyroRobotInit();
	}

	@Override
	public void autonomousInit() {
		resetSensors();	
		gyroAutoInit();
	}

	@Override
	public void autonomousPeriodic() {
		updateSensors();
		gyroAutoPeriodic();

		SmartDashboard.putData("AnglePID", this.anglePIDController);
		SmartDashboard.putNumber("GyroAngleVal: ", gyroAngleVal);
		SmartDashboard.putNumber("GyroRateVal: ", gyroRateVal);
		SmartDashboard.putNumber("LeftEncoderVal: ", leftEncoderVal);
		SmartDashboard.putNumber("RightEncoderVal: ", -rightEncoderVal);
		
	}
	
	@Override
	public void teleopInit() {
		resetSensors();
	}

	@Override
	public void teleopPeriodic() {

		updateSensors();
		driveWithJoysticks(logitechController.getRawAxis(1), logitechController.getRawAxis(5));
		SmartDashboard.putNumber("GyroAngleVal: ", gyroAngleVal);
		SmartDashboard.putNumber("GyroRateVal: ", gyroRateVal);
		SmartDashboard.putNumber("LeftEncoderVal: ", leftEncoderVal);
		SmartDashboard.putNumber("RightEncoderVal: ", -rightEncoderVal);
		SmartDashboard.putNumber("Match Time", matchTime);
		SmartDashboard.putNumber("Battery Charge", battery);
		SmartDashboard.putString("Team Alliance", teamAlliance.toString());//.toString() causes crash
		driverStationDisplay();
	}

	@Override
	public void testPeriodic() {
		
		driveWithJoysticks(logitechController.getRawAxis(1), logitechController.getRawAxis(5));
		SmartDashboard.putNumber("LeftEncoderRate", leftEncoder.getRate());
		
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
	
	public void updateSensors() {
		gyroAngleVal = gyro.getAngle();
		gyroRateVal = gyro.getRate();
		leftEncoderVal = leftEncoder.get();
		rightEncoderVal = rightEncoder.get();
	}
	
	public void gyroRobotInit() {
		
		this.angleKd = 0.12d;
		this.angleKi = 0.0001d;
		this.angleKp = 0.16d;
		this.anglePIDSource = new PIDSource() {
		
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			// TODO Auto-generated method stub
			
		}
		
		@Override
		public double pidGet() {
			Double angle = gyro.getAngle();
			return angle==null ? 0 : angle;
		}
		
		@Override
		public PIDSourceType getPIDSourceType() {
			
			return PIDSourceType.kDisplacement;
		}
	};
	this.anglePIDOutput = new PIDOutput() {
		
		@Override
		public void pidWrite(double output) {
			angleOutput = output;
			
		}
	};
	this.anglePIDController = new PIDController(angleKp, angleKi, angleKd, anglePIDSource, anglePIDOutput);

	}
	
	public void gyroAutoInit() {
		this.anglePIDController.setSetpoint(25);
		this.anglePIDController.setContinuous();
		this.anglePIDController.enable();
	}
	
	public void gyroAutoPeriodic() {
		
		System.out.println(this.angleOutput);
		try{
			frontLeftMotor.set(this.angleOutput * 0.5);
			backLeftMotor.set(this.angleOutput * 0.5);
			frontRightMotor.set(this.angleOutput * 0.5);
			backRightMotor.set(this.angleOutput * 0.5);
		}catch(Exception e){
			
		}
	}
	
	public void encoderRobotInit() {
		encoderPIDSource = new PIDSource() {

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				// TODO Auto-generated method stub
				return null;
			}

			@Override
			public double pidGet() {
				// TODO Auto-generated method stub
				return 0;
			}
			
		};
	}
	
	public void encoderAutonomousInit() {
		
	}
	
	public void driverStationDisplay(){
		this.battery = driverStation.getBatteryVoltage();
		this.teamAlliance = driverStation.getAlliance();
		this.matchTime = driverStation.getMatchTime();  
		
		//ERROR Unhandled exception: java.lang.NullPointerException at [org.usfirst.frc.team5816.robot.Robot.teleopPeriodic(Robot.java:126), edu.wpi.first.wpilibj.IterativeRobot.startCompetition(IterativeRobot.java:130), edu.wpi.first.wpilibj.RobotBase.main(RobotBase.java:247)] 
	}
	

}