 package org.usfirst.frc5933.Ripley2019.commands;

import org.usfirst.frc5933.Ripley2019.Robot;
import org.usfirst.frc5933.Ripley2019.subsystems.DriveTrain;
import org.usfirst.frc5933.Ripley2019.subsystems.RoboRio;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drive straight at the current heading.
 */
public class DriveStraight extends Command {

	// If encoder is on the left, use +1. Eve's is on the R, so -1.  If Ripley's is on the R, use -1
	int encoderS = -1; 

	// if initHeading is near 0 then subsequent gyro readings might roll under to 359 (or vice-versa)
	//  in which case set this to 180
	double initHeadingQuadrantOffset = 0; 
	double initialHeading;
	double initDistance;
	double vBus;
	double threshold;
	double dist;
	double endEnc;
	double initEnc;

	final double kP = 1/320.0 / 4.0;
	double worstYAccel;
	double AccelerometerGetY;
	double agy;

	// slow down at the end of the segment
	boolean slowAtEnd = false; // choosing to slow down at end
	double distMin = 24; // inches
	double vBusSlow = 0.2; 
	double encSlow;
	

	/**
	 * Instantiate a command to drive the robot to a set target
	 * @param vbus
	 * Speed to drive.  Positive for forward, negative for backwards.
	 * @param inches
	 * How far to drive. 
	 * @param threshold
	 * The allowable acceleration to end the command if unexpectedly hit something.  Should be opposite the sign of vbus.
	 * @param sae
	 * Set to true to slow down when near the end of the path segment.
	 */
	public DriveStraight(double vbus, double inches, double threshold, boolean sae) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain); 

		this.vBus = vbus;
		this.dist = Math.abs( inches) * Math.signum(vbus);
		double distSlow = this.dist;
		if( Math.abs( inches) > distMin) {
			distSlow = ( Math.abs( inches) - distMin) * Math.signum(vbus);
		}
		this.endEnc = this.dist * DriveTrain.kEncoderTicksPerInch;
		this.encSlow = distSlow * DriveTrain.kEncoderTicksPerInch;
		this.threshold = threshold;
		this.slowAtEnd = sae;

		this.vBusSlow = this.vBusSlow * Math.signum(vbus);
	}

	// read the encoder actually on the robot, as defined by encoderS
	protected double getRelevantEncoder() {
		if( encoderS > 0) { // Left encoder, Eve
			return Robot.driveTrain.getLeftEncoderPos(0);
		} else {
			return Robot.driveTrain.getRightEncoderPos(0);
		}
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		double h;
		h = Robot.driveTrain.getGyroHeading();
		initHeadingQuadrantOffset = 0;
		if( h < 45 || h > 315) {  // if heading is around zero we'll rotate the compass and modulo 360
			initHeadingQuadrantOffset = 180;
			h = ( h + initHeadingQuadrantOffset) % 360;
		}
		initialHeading = h;

		initEnc = getRelevantEncoder(); 
		AccelerometerGetY = RoboRio.accelerometer.getY();
		endEnc += initEnc;
		encSlow += initEnc;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		double h = ( Robot.driveTrain.getGyroHeading() + initHeadingQuadrantOffset) % 360;
		double proportion = DriveTrain.kPGyroConstant * ( initialHeading - h);
    	double coefficient = 1;
		double v = vBus;
		double enc = getRelevantEncoder();
		
		if( slowAtEnd && ( enc * encSlow > encSlow * encSlow)) {
			v = vBusSlow;
		}

    	//Robot.drivetrain.tankDrive(coefficient * (vBus - proportion), -coefficient * (vBus + proportion)); // from Eve
		Robot.driveTrain.tankDrive(coefficient * (v + proportion), -coefficient * (v - proportion));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		boolean goneFarEnough = false;
		// have we gone far enough?
		if(Math.signum(vBus) < 0) {
			goneFarEnough = getRelevantEncoder() <= endEnc;
		} else {
			goneFarEnough = getRelevantEncoder() >= endEnc;
		}
		// bail when we run into the target... or anything else
		return goneFarEnough || Robot.roboRio.getYAccelerationComparedToThreshold(threshold, true); 
		
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrain.tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}