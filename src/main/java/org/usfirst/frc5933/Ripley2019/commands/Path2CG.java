 package org.usfirst.frc5933.Ripley2019.commands;

import org.usfirst.frc5933.Ripley2019.Robot;
//import org.usfirst.frc5933.Ripley2019.SocketVisionSender;
import org.usfirst.frc5933.Ripley2019.subsystems.DriveTrain;
import org.usfirst.frc5933.Ripley2019.subsystems.RoboRio;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Path2CG extends CommandGroup {

	int encoderS = 1; // Eve's encoder is on the left, use +1. Ripley's is on the R, use -1
	
	final double kP = 1/320.0 / 4.0; // P coefficient in PID. Identify something that works for the robot.
	final double degPerTickCoeff = 0.001; // identify empirically for the robot

	double initEncoder;
	double degPerTick; // degrees per tick (20 msec) the robot should turn along this arc
	double lastHeading; // keep the gyro reading from one tick (execute()) to the next
	double distR; // distance the right wheel of the robot will travel to complete the desired arc path
	double distL; // distance the left wheel of the robot will travel to complete the desired arc path
	double vBusR; // baseline voltage for the right wheel
	double vBusL; // baseline voltage for the left wheel
	double threshold; // stop if we run into anything

	
	/**
	 * Path from right front cargo ship to back up then aim at the right loading station.
	 */
	public Path2CG() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain); 

				// Add Commands here:
		// e.g. addSequential(new Command1());
		//      addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		//      addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.
		
		// Starting with the wheels @ front edge of lvl 2 on right side with the edge of the robot with the outer edge of platform
		// drive straight gyro for 38 "
		//addSequential(new DriveStraight(0.7, 38, -0.6, true));


		//Starting w/ the wheels @ middle wheel front edge of platform on right side

		//turn right forward 67 Degrees 49 in radius
		addSequential( new DriveArc( 0.7, 49, 67, -1.1));

		// turn left forward 23 degrees 49 in radius
		addSequential( new DriveArc( 0.7, 49, -45, -1.1)); 

		// drive straight 58 in 
		//addSequential(new DriveStraight(0.7, 58, -0.6, true));

		//turn left forward 90 degreess radius 28 in
		//addSequential(new DriveArc(0.7, 28, 90, -0.6));

		// drive straight vision (fingers crossed)
		//addSequential(new DriveStraightVision(0.7, "WE ARE AWESOME", -0.6));

		

		//
		// testing driveArc:
		//
		// 3/3/19 driving forward right.
		// addSequential( new DriveArc( 0.7, 40, 45, -1.6));

		// 3/3/19 driving forward left.
		//addSequential( new DriveArc( 0.7, 40, -45, -1.1)); 

		// 3/3/19 driving backward right
		// addSequential( new DriveArc( -0.7, 40, 45, 1.6));

		// 3/3/19 driving backward left
		// addSequential( new DriveArc( -0.7, 40, -45, 1.6));

		

		/*
		// back up 180deg to right, 47" radius... but using 20% less empirically
		addSequential( new DriveArc( -0.7, 40, 180, 1.1));
		// slow down as we complete the back up turn right. all numbers empirical
		addSequential( new DriveArc( -0.4, 20, 60, 1.1)); 
		// forward 45deg to left, 27" radius. empirical
		addSequential( new DriveArc( 0.7, 27, -50, -1.1)); 
		// fwd 45deg to right, 27" radius. empirical
		addSequential( new DriveArc( 0.7, 37, 90, -1.1)); 
		// drive straight in to pick up the disk
		addSequential( new DriveStraightVision( 0.6, "MrHall is Awesome", -0.6)); // rest of the way in
		*/
	}

}