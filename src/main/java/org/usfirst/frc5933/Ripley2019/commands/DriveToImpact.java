/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc5933.Ripley2019.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc5933.Ripley2019.Robot;
import org.usfirst.frc5933.Ripley2019.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveToImpact extends Command {

  double initHeading;
  double vBus;
  double threshold;
  double lastAccel;

  public DriveToImpact(double voltageBus, double accelerationToStop) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);

    vBus = voltageBus;
    threshold = accelerationToStop;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initHeading = Robot.driveTrain.getGyroHeading();
    SmartDashboard.putString("Current Command: ", "DriveToImpact");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double proportion = DriveTrain.kPGyroConstant * (Robot.driveTrain.getGyroHeading() - initHeading);
    Robot.driveTrain.tankDrive((vBus - proportion), -(vBus + proportion));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.roboRio.getYAccel() < (threshold) * -Math.signum(vBus); 
   }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.tankDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
