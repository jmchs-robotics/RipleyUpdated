 package org.usfirst.frc5933.Ripley2019.commands;

import org.usfirst.frc5933.Ripley2019.Robot;
import org.usfirst.frc5933.Ripley2019.subsystems.DriveTrain;
import org.usfirst.frc5933.Ripley2019.subsystems.RoboRio;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveStraightVision extends Command {

	// if initHeading is near 0 then subsequent gyro readings might roll under to 359 (or vice-versa)
	//  in which case set this to 180
	double initHeadingQuadrantOffset = 0; 
	double initHeading;
	double initDistance;
	double vBus;
	String vision;
	double threshold;

	final double kP = 1/320.0 / 4.0; // 4.0;  // 4.0 worked well for Eve, practice at Waterford on 3/2/19
	double worstYAccel;
	double AccelerometerGetY;
	double agy;

	// slow down at the end of the approach to the target
	// count times we actually see the target with the vision co-processor
	// if we've seen the target enough > vTargetCountMin and we're within a few feet < vDist slow down to vBusSlow
	boolean slowAtEnd = false; // Set to true to enable slowing down at end.
	int vTargetCount = 0;
	int vTargetCountMin = 10;
	double vDistMin = 60; // inches, compared to distance given by vision processor
	double vBusSlow = 0.2; 
	boolean slowSpeed = false; // gets set to true once we're within vDistMin and targetCount > vTargetCountMin

	/**
	 * Instantiate a command to drive the robot to a set target
	 * @param vbus
	 * Speed to drive forward
	 * @param visionType
	 * The {@link SocketVisionSender} constant string to use
	 * @param threshold
	 * The allowable acceleration to end the command.
	 */
	public DriveStraightVision(double vbus, String visionType, double threshold) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain); 

		this.vision = visionType;
		this.vBus = vbus;
		this.threshold = threshold;
	}

	/**
	 * Instantiate a command to drive the robot to a set target
	 * @param vbus
	 * Speed to drive forward
	 * @param visionType
	 * The {@link SocketVisionSender} constant string to use
	 * @param threshold
	 * The allowable acceleration to end the command.
	 * @param sae
	 * Set to true if you want the robot to slow down as it approaches a previously identified target
	 */
	public DriveStraightVision( double vbus, String visionType, double threshold, boolean sae) {
		requires(Robot.driveTrain); 
		this.slowAtEnd = sae;

		this.vision = visionType;
		this.vBus = vbus;
		this.threshold = threshold;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		//Robot.sender_.setSendData(vision);

		initHeading = Robot.driveTrain.getGyroHeading();
		AccelerometerGetY = RoboRio.accelerometer.getY();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		double error = -0.001;
		double proportion = 0;
		double coefficient = 0.4;
		double d = 0;
		double h = 0;

		if(AccelerometerGetY <= 0)
		{
			worstYAccel = AccelerometerGetY;
		}
		SmartDashboard.putNumber("WorstYAccel", worstYAccel);

		// tracking to the RFT target
		error = Robot.rft_.get_degrees_x();  // If the robot needs to turn more rightward, this number is more positive

		// if the vision didn't provide a new value or isn't finding anything, drive straight based on gyro reading
		if(( error == 0) || ( error > -0.0011 && error < -0.0009)  // no new answer from vision
			|| ( error > -0.011 && error < -0.009)  // vision didn't find any contours
			|| ( error > -0.031 && error < -0.029)) { // vision didn't find any targets
			h = Robot.driveTrain.getGyroHeading();  // gyro reading is more positive when the robot turns more to the right
			// h = ( Robot.driveTrain.getGyroHeading() + initHeadingQuadrantOffset) % 360;
			proportion = DriveTrain.kPGyroConstant * ( initHeading - h);

		} else { 
			// drive using error provided by vision
			proportion = error * kP;
			//if(error == 0) initHeading = Robot.driveTrain.getGyroHeading();
			// if we lose vision, we'll keep going striaght in this direction
			h = Robot.driveTrain.getGyroHeading();
			/* 
			
			initHeadingQuadrantOffset = 0;
			if( h < 45 || h > 315) {  // if heading is around zero we'll rotate the compass and modulo 360
				initHeadingQuadrantOffset = 180;
				h = ( h + initHeadingQuadrantOffset) % 360;
			}*/
			initHeading = h;

			// keep track in case want to slow down close to target
			vTargetCount ++;
			if( vTargetCount > vTargetCountMin && slowAtEnd) {
				if( Robot.rft_.get_distance() < vDistMin) {
					slowSpeed = true;
				}
			}
		}

		coefficient = 1;
		// either run at user's desired speed or at slow speed
		double v = vBus;
		if( slowSpeed) {
			v = vBusSlow;
		}
		
		Robot.driveTrain.tankDrive(coefficient * (v + proportion), -coefficient * (v - proportion));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// bail when we run into the target... or anything else
		return Robot.roboRio.getYAccelerationComparedToThreshold(threshold, true); 
		//|| initDistance - Robot.driveTrain.getRightEncoderPos(0) < 18 * DriveTrain.kEncoderTicksPerInch; 
	}

	// Called once after isFinished returns true
	protected void end(){
		Robot.driveTrain.tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}