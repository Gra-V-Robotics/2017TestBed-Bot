package org.usfirst.frc.team5816.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	Encoder leftEncoder = new Encoder(0, 1, true, EncodingType.k4X);
	Encoder rightEncoder = new Encoder(2, 3, true, EncodingType.k4X);

	Gyro gyro = new ADXRS450_Gyro();

	Joystick logitechController = new Joystick(0);

//	AHRS navX = new AHRS(SerialPort.Port.kMXP);

	CANTalon frontLeftMotor = new CANTalon(4);
	CANTalon frontRightMotor = new CANTalon(2);
	CANTalon backLeftMotor = new CANTalon(5);
	CANTalon backRightMotor = new CANTalon(3);
	CANTalon intakeMotor = new CANTalon(6);

	double gyroAngleVal;
	double gyroRateVal;
	double leftEncoderVal;
	double rightEncoderVal;

	boolean buttonVal;

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

//	Shooter shooter;
	TergraHandler tergraHandler;
	
//	Boolean autoRun = false;

	long camMiliTime;
	
	@Override
	public void robotInit() {
		sensorCalibration();
		gyroRobotInit();
//		this.shooter = new Shooter();
		this.tergraHandler = new TergraHandler();

	}
	
	public void prints(){
		System.out.println("Gyro: "+gyro.getAngle());
		System.out.println("V: "+ this.tergraHandler.shooterIsVisible());
		System.out.println("FA: " + this.tergraHandler.getShooterAngle());
	}

	@Override
	public void autonomousInit() {
		this.prints();
		resetSensors();	
		this.prints();
		if(this.tergraHandler.shooterIsVisible()){
			this.anglePIDController.setSetpoint(gyro.getAngle() + this.tergraHandler.getShooterAngle());
			this.anglePIDController.setContinuous();
			this.anglePIDController.enable();
		}else{
			gyroAutoInit();
		}
		this.camMiliTime = System.currentTimeMillis();
//		gyroAutoInit();
		
	}

	@Override
	public void autonomousPeriodic() {
		updateSensors();
		gyroAutoPeriodic();
//		this.autoRun = SmartDashboard.getBoolean("AutoRun", false);
		SmartDashboard.putData("AnglePID", this.anglePIDController);
		SmartDashboard.putNumber("GyroAngleVal: ", gyroAngleVal);
		SmartDashboard.putNumber("GyroRateVal: ", gyroRateVal);
		SmartDashboard.putNumber("LeftEncoderVal: ", leftEncoderVal);
		SmartDashboard.putNumber("RightEncoderVal: ", -rightEncoderVal);
		
		if(((System.currentTimeMillis() - this.camMiliTime) > 500) && this.tergraHandler.shooterIsVisible()){
			this.anglePIDController.setSetpoint(gyro.getAngle() + this.tergraHandler.getShooterAngle());
			this.camMiliTime = System.currentTimeMillis();
		}
		

	}

	@Override
	public void teleopInit() {
		resetSensors();
	}

	@Override
	public void teleopPeriodic() {

		updateSensors();
		//navXStats();

		driveWithJoysticks(logitechController.getRawAxis(1), logitechController.getRawAxis(3));

		if(logitechController.getRawButton(4)){
			intakeMotor.set(1.0);
		}else{
			intakeMotor.set(0.0);
		}

		SmartDashboard.putNumber("GyroAngleVal: ", gyroAngleVal);
		SmartDashboard.putNumber("GyroRateVal: ", gyroRateVal);
		SmartDashboard.putNumber("LeftEncoderVal: ", leftEncoderVal);
		SmartDashboard.putNumber("RightEncoderVal: ", -rightEncoderVal);
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

		this.angleKd = 0.1d;
		this.angleKi = 0.0005d;
		this.angleKp = 0.05;
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
		this.anglePIDController.setSetpoint(0);
		this.anglePIDController.setContinuous();
		this.anglePIDController.enable();
	}

	public void gyroAutoPeriodic() {

//		System.out.println(this.angleOutput);
		try{
			frontLeftMotor.set(this.angleOutput * 0.25);
			backLeftMotor.set(this.angleOutput * 0.25);
			frontRightMotor.set(this.angleOutput * 0.25);
			backRightMotor.set(this.angleOutput * 0.25);
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

	public void navXStats() {
//		SmartDashboard.putNumber("Heading: ", navX.getCompassHeading());
//		SmartDashboard.putNumber("Angle: ", navX.getAngle());
//		SmartDashboard.putNumber("Rate: ", navX.getRate());
//		SmartDashboard.putNumber("X Displacement: ", navX.getDisplacementX());
//		SmartDashboard.putNumber("Y Displacement: ", navX.getDisplacementY());
//		SmartDashboard.putNumber("Z Displacement: ", navX.getDisplacementZ());
//		SmartDashboard.putNumber("Pitch: ", navX.getPitch());
//		SmartDashboard.putNumber("Roll: ", navX.getRoll());
//		SmartDashboard.putNumber("Yaw: ", navX.getYaw());
//		SmartDashboard.putNumber("Temp: ", navX.getTempC());
//		SmartDashboard.putNumber("Velocity X: ", navX.getVelocityX());
//		SmartDashboard.putNumber("Velocity Y: ", navX.getVelocityY());
//		SmartDashboard.putNumber("Velocity Z: ", navX.getVelocityZ());
	}

	public void driveWithEncoderJoystick(int leftEncoderVal, int rightEncoderVal, double leftDriveVal, double rightDriveVal) {
		if (leftDriveVal == rightDriveVal) {

		}
	}

}