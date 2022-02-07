/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc5933.Ripley2019.commands;

import org.usfirst.frc5933.Ripley2019.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class YeetArmBackwardTimed extends Command {
  //This command is for the autonomous End game climb... in theory
  public YeetArmBackwardTimed(double time) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    
    requires(Robot.yeetArm);

    setTimeout(time); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.yeetArm.setSpeed(-0.7);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    isTimedOut(); {
      return true;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.yeetArm.stopYeet();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
