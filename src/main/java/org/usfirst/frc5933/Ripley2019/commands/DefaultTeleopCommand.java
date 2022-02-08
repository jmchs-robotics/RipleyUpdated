// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5933.Ripley2019.commands;
import edu.wpi.first.wpilibj.command.Command; 
import org.usfirst.frc5933.Ripley2019.Robot;
import edu.wpi.first.wpilibj.smartdashboard.*;
import org.usfirst.frc5933.Ripley2019.subsystems.DriveTrain;


/**
 *
 */
public class DefaultTeleopCommand extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public DefaultTeleopCommand() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveTrain);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.driveTrain.tankDrive(); 
        SmartDashboard.putString("DefaultTeleop Running: ", " execute");
        try {
            SmartDashboard.putString( "Vision Socket direction:", Robot.rft_.get_direction());
            SmartDashboard.putNumber( "Vision Socket dir offset:", Robot.rft_.get_degrees_x());
            SmartDashboard.putNumber( "Vision Socket distance:", Robot.rft_.get_distance());
        } catch (Exception e) {
            SmartDashboard.putString( "Vision Socket direction:", "ERROR Robot.rft_ problem: " + e.toString());
            SmartDashboard.putNumber( "Vision Socket dir offset:", -100);
            SmartDashboard.putNumber( "Vision Socket distance:", -100);
        }

        SmartDashboard.putNumber( "Left encoder:", Robot.driveTrain.getLeftEncoderPos(0));
        SmartDashboard.putNumber( "Right encoder:", Robot.driveTrain.getRightEncoderPos(0));

        //Get triggers from subsystem joystick to control cargo
        /*Robot.oi.getSubsystemJoystick().setThrottleChannel(2);
        Robot.cargo.setCargoMotorSpeed(-Robot.oi.getSubsystemJoystick().getThrottle());
        Robot.oi.getSubsystemJoystick().setThrottleChannel(3);
        Robot.cargo.setCargoMotorSpeed(Robot.oi.getSubsystemJoystick().getThrottle());
        SmartDashboard.putNumber("Throttle Channel:",Robot.oi.getSubsystemJoystick().getThrottleChannel());
       
         //Get triggers from subsystem joystick to control shudderLeft and shudderRight
         Robot.oi.getDriverJoystick().setThrottleChannel(2);
         Robot.driveTrain.shudderLeft(Robot.oi.getDriverJoystick().getThrottle());
         Robot.oi.getDriverJoystick().setThrottleChannel(3);
         Robot.driveTrain.shudderRight(Robot.oi.getDriverJoystick().getThrottle());
         SmartDashboard.putNumber("Throttle Channel:",Robot.oi.getDriverJoystick().getThrottleChannel()); 
         */
         

        //Arm arm
        //Robot.yeetArm.setSpeed(Robot.oi.getSubsystemJoystick().getY());
        //Robot.arm.moveArm();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.stop();

        SmartDashboard.putString("DefaultTeleop Running: ", "just finished");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end(); 
    }
}
