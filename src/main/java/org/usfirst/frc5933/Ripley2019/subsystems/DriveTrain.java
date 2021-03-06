// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prRipleynt
// it from being updated in the future.


package org.usfirst.frc5933.Ripley2019.subsystems;

import org.usfirst.frc5933.Ripley2019.Robot;
import org.usfirst.frc5933.Ripley2019.RobotMap;
import org.usfirst.frc5933.Ripley2019.commands.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StickyFaults;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.smartdashboard.*;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class DriveTrain extends Subsystem {
	//Drivetrain constants
	
	//Field-tested value. Seems to work well.... or we built an awesome drivetrain this year.
	//But it's probably the software.
	public static final double kPGyroConstant = 0.01;
	public static final double kPGyroTurnConstant = 0.01;
	
	/**<p>
	 * Encoder runs 4096 steps/revolution
	 * Wheel is 4" diameter; encoder is attached directly to output shaft
	 * therefore, wheel moves 4π"/revolution.
	 * 4π"/4096 steps ≈ π"/1024 steps ≈ 0.0030679616"/step. </p>
	 * Also, 4096 steps/4π" ≈ 325.9493209 steps/inch.
	 * </p>
	 * One can take a value in inches and multiply by kEncoderTicksPerInch
	 * to get the number of encoder ticks needed to reach that distance in inches.
	 */
	public static final double kEncoderTicksPerInch = 325.9493209;
	
	//PID constants
	private static final double kLeftP = 0;
	private static final double kLeftI = 0;
	private static final double kLeftD = 0;
	private static final double kLeftF = 0;
	
	private static final double kRightP = 0;
	private static final double kRightI = 0;
	private static final double kRightD = 0;
	private static final double kRightF = 0;
	private static final double minVBusOutVal = 0.2;

	private static double shudderMagnitude = 0.6; //arbitrary number...for testing
	private static boolean leftShudderLatch = false;
	private static boolean rightShudderLatch = false;

	
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private WPI_TalonSRX driveTrainLeftMaster;
    private WPI_TalonSRX driveTrainRightMaster;
    private DifferentialDrive robotDrive;
    private WPI_TalonSRX driveTrainLeftFollower1;
    private WPI_TalonSRX driveTrainRightFollower1;
    private WPI_TalonSRX driveTrainLeftFollower2;
    private WPI_TalonSRX driveTrainRightFollower2;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    private ADXRS450_Gyro gyro; //  = RobotMap.Gyro;

	@Override
	public void initDefaultCommand() {
    
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new DefaultTeleopCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void init() {
		gyro = new ADXRS450_Gyro(); // RobotMap.Gyro;

        driveTrainLeftMaster = RobotMap.drivetrainLeftMaster;
        driveTrainRightMaster = RobotMap.drivetrainRightMaster;
        driveTrainLeftFollower1 = RobotMap.drivetrainLeftFollower1;
	    driveTrainLeftFollower2 = RobotMap.drivetrainLeftFollower2;
	    driveTrainRightFollower1 = RobotMap.drivetrainRightFollower1;
        driveTrainRightFollower2 = RobotMap.drivetrainRightFollower2;
        
        robotDrive = new DifferentialDrive(driveTrainLeftMaster, driveTrainRightMaster);
	
        setNominalAndMax();
		initGyro();
		setFollower();
		configMasterFeedback(0, 0);
		
		setVoltageFullOut(12, 0);
		
		driveTrainLeftMaster.set(ControlMode.PercentOutput, 0);
		driveTrainRightMaster.set(ControlMode.PercentOutput, 0);
	}

	public void teleopInit() {
		setCurrentLimit(0,0,0,10);
		setBrakeMode(false);
		enableCurrentLimit(true);
	
		driveTrainLeftMaster.set(ControlMode.PercentOutput, 0);
		driveTrainRightMaster.set(ControlMode.PercentOutput, 0);
	
	}

	public void autonomousInit() {
		setCurrentLimit(0,0,0,10);
		setBrakeMode(true);
		enableCurrentLimit(true);
	
		driveTrainLeftMaster.set(ControlMode.PercentOutput, 0);
		driveTrainRightMaster.set(ControlMode.PercentOutput, 0);
	}

	@Override
	public void periodic() {
		// Put code here to be run Ripleyry loop

		// SmartDashboard.putNumber("Joystick X: ", Robot.oi.driverJoystick.getX());
		// SmartDashboard.putNumber("Joystick Y: ", Robot.oi.driverJoystick.getY());
		// SmartDashboard.putNumber("Joystick Magnitude: ", Robot.oi.driverJoystick.getMagnitude());
		
		// SmartDashboard.putNumber("Left Out: ", driveTrainLeftMaster.getMotorOutputPercent());
		// SmartDashboard.putNumber("Right Out: ", driveTrainRightMaster.getMotorOutputPercent());
		
		// SmartDashboard.putNumber("Left Enc: ", getLeftEncoderPos(0));
		// SmartDashboard.putNumber("Right Enc: ", getRightEncoderPos(0));
		// SmartDashboard.putNumber("Gyro Out: ", getGyroHeading());
	}

	private void setNominalAndMax() {
		configPeakOutput(driveTrainLeftMaster, 1.0f, -1.0f);
		configNominalOutput(driveTrainLeftMaster, +0.0f, -0.0f);
		configPeakOutput(driveTrainLeftFollower1, 1.0f, -1.0f);
		configNominalOutput(driveTrainLeftFollower1, +0.0f, -0.0f);
		configPeakOutput(driveTrainLeftFollower2, 1.0f, -1.0f);
		configNominalOutput(driveTrainLeftFollower2, +0.0f, -0.0f);
		
		configPeakOutput(driveTrainRightMaster, 1.0f, -1.0f);
		configNominalOutput(driveTrainRightMaster, +0.0f, -0.0f);
		configPeakOutput(driveTrainRightFollower1, 1.0f, -1.0f);
		configNominalOutput(driveTrainRightFollower1, +0.0f, -0.0f);
		configPeakOutput(driveTrainRightFollower2, 1.0f, -1.0f);
		configNominalOutput(driveTrainRightFollower2, +0.0f, -0.0f);
	}
	
	private void setVoltageFullOut(double voltage, int timeout) {
		driveTrainLeftMaster.configVoltageCompSaturation(voltage, timeout);
		driveTrainRightMaster.configVoltageCompSaturation(voltage, timeout);
		driveTrainLeftFollower1.configVoltageCompSaturation(voltage, timeout);
		driveTrainRightFollower1.configVoltageCompSaturation(voltage, timeout);
		driveTrainLeftFollower2.configVoltageCompSaturation(voltage, timeout);
		driveTrainRightFollower2.configVoltageCompSaturation(voltage, timeout);
	}

	private void configPeakOutput(WPI_TalonSRX controller, double percentForward, double percentRRipleyrse) {
		controller.configPeakOutputForward(percentForward, 0);
		controller.configPeakOutputReverse(percentRRipleyrse, 0);
	}

	private void configNominalOutput(WPI_TalonSRX controller, double percentForward, double percentRRipleyrse) {
		controller.configNominalOutputForward(percentForward, 0);
		controller.configNominalOutputReverse(percentRRipleyrse, 0);
	}

	private void setBrakeMode(boolean brake) {
		if(brake) {
			driveTrainLeftMaster.setNeutralMode(NeutralMode.Brake);
			driveTrainRightMaster.setNeutralMode(NeutralMode.Brake);
			driveTrainLeftFollower1.setNeutralMode(NeutralMode.Brake);
			driveTrainRightFollower1.setNeutralMode(NeutralMode.Brake);
			driveTrainLeftFollower2.setNeutralMode(NeutralMode.Brake);
			driveTrainRightFollower2.setNeutralMode(NeutralMode.Brake);
		}else {
			driveTrainLeftMaster.setNeutralMode(NeutralMode.Coast);
			driveTrainRightMaster.setNeutralMode(NeutralMode.Coast);
			driveTrainLeftFollower1.setNeutralMode(NeutralMode.Coast);
			driveTrainRightFollower1.setNeutralMode(NeutralMode.Coast);
			driveTrainLeftFollower2.setNeutralMode(NeutralMode.Coast);
			driveTrainRightFollower2.setNeutralMode(NeutralMode.Coast);
		}
	}
	
	public void configMasterFeedback(int pidIndx, int timeoutMs) {
		driveTrainLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidIndx, timeoutMs);
		configMasterPID(driveTrainLeftMaster, 0, kLeftP, kLeftI, kLeftD, kLeftF, timeoutMs);
		
		driveTrainRightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidIndx, timeoutMs);
		configMasterPID(driveTrainRightMaster, 0, kRightP, kRightI, kRightD, kRightF, timeoutMs);
	}
	
	public void configMasterPID(WPI_TalonSRX masterController, int pidIdx, double p, double i, double d, double f, int timeoutMs) {
		masterController.config_kP(pidIdx, p, timeoutMs);
		masterController.config_kI(pidIdx, i, timeoutMs);
		masterController.config_kD(pidIdx, d, timeoutMs);
		masterController.config_kF(pidIdx, f, timeoutMs);
	}

	/**
	 * For simpler control, set all values to 0 except the continuousLimit parameter. The timeout is
	 * irrelevant right now because the method is void: it just swallows any returned errors.
	 * @param
	 * msErrorTimeout
	 * The timeout (in ms) to pause each config to wait for confirmation/errors
	 * @param
	 * peakCurrent
	 * The maximum allowable current per motor controller for the peakDuration
	 * @param
	 * peakDuration
	 * The time (in ms) to allow the motor controllers to pull the peakCurrent
	 * @param
	 * continuousLimit
	 * The current limit for Ripleyry time other than the peak.
	 */
	private void setCurrentLimit(int msErrorTimeout, int peakCurrent, int peakDuration, int continuousLimit) {
		driveTrainLeftMaster.configPeakCurrentLimit(peakCurrent, msErrorTimeout);
		driveTrainLeftMaster.configPeakCurrentDuration(peakDuration, msErrorTimeout);
		//again, it's amps then timeout for error reporting. We don't want to wait so much.
		driveTrainLeftMaster.configContinuousCurrentLimit(continuousLimit, msErrorTimeout);
		
		driveTrainLeftFollower1.configPeakCurrentLimit(peakCurrent, msErrorTimeout);
		driveTrainLeftFollower1.configPeakCurrentDuration(peakDuration, msErrorTimeout);
		//again, it's amps then timeout for error reporting. We don't want to wait so much.
		driveTrainLeftFollower1.configContinuousCurrentLimit(continuousLimit, msErrorTimeout);
		
		driveTrainLeftFollower2.configPeakCurrentLimit(peakCurrent, msErrorTimeout);
		driveTrainLeftFollower2.configPeakCurrentDuration(peakDuration, msErrorTimeout);
		//again, it's amps then timeout for error reporting. We don't want to wait so much.
		driveTrainLeftFollower2.configContinuousCurrentLimit(continuousLimit, msErrorTimeout);

		driveTrainRightMaster.configPeakCurrentLimit(peakCurrent, msErrorTimeout);
		driveTrainRightMaster.configPeakCurrentDuration(peakDuration, msErrorTimeout);
		//again, it's amps then timeout for error reporting. We don't want to wait so much.
		driveTrainRightMaster.configContinuousCurrentLimit(continuousLimit, msErrorTimeout);
		
		driveTrainRightFollower1.configPeakCurrentLimit(peakCurrent, msErrorTimeout);
		driveTrainRightFollower1.configPeakCurrentDuration(peakDuration, msErrorTimeout);
		//again, it's amps then timeout for error reporting. We don't want to wait so much.
		driveTrainRightFollower1.configContinuousCurrentLimit(continuousLimit, msErrorTimeout);
		
		driveTrainRightFollower2.configPeakCurrentLimit(peakCurrent, msErrorTimeout);
		driveTrainRightFollower2.configPeakCurrentDuration(peakDuration, msErrorTimeout);
		//again, it's amps then timeout for error reporting. We don't want to wait so much.
		driveTrainRightFollower2.configContinuousCurrentLimit(continuousLimit, msErrorTimeout);
	}
	
	private void enableCurrentLimit(boolean enable) {
		driveTrainLeftMaster.enableCurrentLimit(enable);
	    driveTrainRightMaster.enableCurrentLimit(enable);
		driveTrainRightFollower1.enableCurrentLimit(enable);
		driveTrainLeftFollower1.enableCurrentLimit(enable);
		driveTrainRightFollower2.enableCurrentLimit(enable);
		driveTrainLeftFollower2.enableCurrentLimit(enable);
	}

	private void setFollower() {
		driveTrainLeftFollower1.set(ControlMode.Follower, 10);
		driveTrainLeftFollower2.set(ControlMode.Follower, 10);

		driveTrainRightFollower1.set(ControlMode.Follower, 11);
		driveTrainRightFollower2.set(ControlMode.Follower, 11);
	}
	
	public double getLeftEncoderPos(int pidIdx) {
        return driveTrainLeftMaster.getSelectedSensorPosition(pidIdx);
	}
	
	public double getRightEncoderPos(int pidIdx) {
		return driveTrainRightMaster.getSelectedSensorPosition(pidIdx);
	}

	public void arcadeDrive() {
		//robotDrive.arcadeDrive( Robot.oi.driverJoystick.getX(), Robot.oi.driverJoystick.getY()); feels like 90 deg off
		robotDrive.arcadeDrive(-Robot.oi.driverJoystick.getLeftY(), Robot.oi.driverJoystick.getLeftX());
	}
	
	public void tankDrive(double leftVal, double rightVal) {
		driveTrainLeftMaster.set(leftVal);
		driveTrainRightMaster.set(rightVal);
	}

	public void tankDrive() {
		driveTrainLeftMaster.set(-Robot.oi.driverJoystick.getLeftY());
		driveTrainRightMaster.set(-Robot.oi.driverJoystick.getRightY());
	}
	
	
	//goodies for gyro
	private void initGyro() {
		gyro.calibrate();
	}
	
	public void resetGyro() {
		gyro.reset();
	}
	
	public double getGyroHeading() {
		return gyro.getAngle();
	}

	public double thresholdVBus(double val) {
		if(Math.abs(val) < minVBusOutVal) {
			val = Math.signum(val) * minVBusOutVal;
		}
		return val;
	}

	public void resetEncoders() {
		driveTrainLeftMaster.setSelectedSensorPosition(0, 0, 20);
		driveTrainRightMaster.setSelectedSensorPosition(0, 0, 20);
    }
    
    public void stop() {
		robotDrive.stopMotor();
	}

	public void shudderLeft(double triggerVal) {
		if (triggerVal > 0.5 && leftShudderLatch == false) {
			tankDrive(-shudderMagnitude, -shudderMagnitude);
			leftShudderLatch = true;
		} else if ( triggerVal < 0.4) {
			leftShudderLatch = false;
		}
		SmartDashboard.putNumber("trigger Value", triggerVal);
	}
	
	public void shudderRight(double triggerVal) {
		if (triggerVal > 0.5 && rightShudderLatch == false) {
			tankDrive(shudderMagnitude, shudderMagnitude);
			rightShudderLatch = true; 
		} else if ( triggerVal < 0.4) {
			rightShudderLatch = false;
		}
	}
}