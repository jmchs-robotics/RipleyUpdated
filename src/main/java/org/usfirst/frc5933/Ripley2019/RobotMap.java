// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// Test...Test.
//Testing again
//test
//another change


package org.usfirst.frc5933.Ripley2019;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	//drivetrain master objects
	public static WPI_TalonSRX drivetrainLeftMaster;
    public static WPI_TalonSRX drivetrainRightMaster;
    public static DifferentialDrive drivetrainRobotDrive;

    //drivetrain follower objects
    public static WPI_TalonSRX drivetrainLeftFollower1;
    public static WPI_TalonSRX drivetrainLeftFollower2;
    public static WPI_TalonSRX drivetrainRightFollower1;
    public static WPI_TalonSRX drivetrainRightFollower2;
    
    //control system objects
    //public static ADXRS450_Gyro Gyro;
    
    //arm manipulation objects
    public static WPI_TalonSRX armSubsystemMotor;

    //cargo manipulation objects
    public static Spark cargoSubsystemMotor;
//    public static DoubleSolenoid cargoSubsystemSolenoid;

    //end game objects
    public static Spark endgameSubsystemMotor;
//    public static DoubleSolenoid endgameSubsystemSolenoid;

    //hatch manipulation objects
//    public static DoubleSolenoid hatchSubsystemSolenoid;

    //wrist manipulation objects
//    public static DoubleSolenoid wristSubsystemSolenoid;
    
    //roborio objects
    // public static DigitalInput[] DIPs;
    public static BuiltInAccelerometer Accelerometer;

    //camera
    public static CameraServer server;
    
    public static void init() {
    	//Drivetrain master instantiation.
    	//Convention: evens are left motors, odds are right motors.
        drivetrainLeftMaster = new WPI_TalonSRX(10);
        drivetrainRightMaster = new WPI_TalonSRX(11);
        
        //Instantiate robot drive type for teleop control.
        drivetrainRobotDrive = new DifferentialDrive(drivetrainLeftMaster, drivetrainRightMaster);
        drivetrainRobotDrive.setSafetyEnabled(true);
        // the "DifferentialDrive... Output not updated often enough." error may only happen when Talons aren't connected to motors.
        // we can stop the "DifferentialDrive... Output not updated enough." error by setting setSafetyEnabled(false)
        drivetrainRobotDrive.setSafetyEnabled(true);
        // we can slow (but not stop) the rate of the "DifferentialDrive... Output not updated enough." error by setting setExpiration(10)
        drivetrainRobotDrive.setExpiration(0.1);
        drivetrainRobotDrive.setMaxOutput(1.0);
        
        //instantiate left side followers on even IDs
        drivetrainLeftFollower1 = new WPI_TalonSRX(12);
        drivetrainLeftFollower2 = new WPI_TalonSRX(14);
        
        //instantiate right side followers on odd IDs
        drivetrainRightFollower1 = new WPI_TalonSRX(13);
        drivetrainRightFollower2 = new WPI_TalonSRX(15);
        
        //instantiate gyro. B/c it is an SPI gyroscope, no need for calibration methods yet
        //Gyro = new ADXRS450_Gyro();
        
        //instantiate PCM doublesolenoid for the double piston starting from ID 0, slot 0;
//        armTongsPiston = new DoubleSolenoid(0, 0, 1);
        
        //instantiate the array of digital inputs on the RIO. This contains all accessible DIPs (a bit overkill,
        //but useful for planning ahead).
        // DIPs = new DigitalInput[10];
        
        //create each individual dip.
        // for(int i = 0; i < DIPs.length; i ++) {
        // 	DIPs[i] = new DigitalInput(i);
        // }
        
        //instantiate the accelerometer present in all RIOs. On the RIO casing, the arrows point to the positive direction.
        Accelerometer = new BuiltInAccelerometer();
    
        //instantiate the end game actuators (a piston and Spark) on next available slots.
//        EndGamePiston = new DoubleSolenoid(0, 2, 3);
        endgameSubsystemMotor = new Spark(9);

        //instantiate the cameraserver.
        server = CameraServer.getInstance();
    }
}
