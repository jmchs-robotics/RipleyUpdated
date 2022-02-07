/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc5933.Ripley2019.commands;

import org.usfirst.frc5933.Ripley2019.Robot;
import org.usfirst.frc5933.Ripley2019.subsystems.DriveTrain;
import org.usfirst.frc5933.Ripley2019.subsystems.RoboRio;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;

public class EndGameClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  @Override
  protected synchronized void requires(Subsystem subsystem) {
  }
  public EndGameClimb() {
    requires(Robot.yeetArm);
    requires(Robot.cricketLegs);
    requires(Robot.driveTrain);
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    addSequential(new YeetArmForwardTimed(1.25));
    //It took the yeetArm 3 seconds to be in position for the cricket legs to deploy

    addSequential(new CricketLegsOut());

    addParallel(new YeetArmForwardTimed(5.0)); // not sure on that time
   
    addSequential(new DriveStraight(-0.7,10.0,1.0,true));

    addParallel(new DriveStraight(-0.7, 60.0,0.6, true));
    
    addSequential(new CricketLegsIn());

    //addSequential(new Wait(1.5));

    //addSequential(new DriveStraight(-0.7,60.0,0.6,true)); //definitly check on the threshold value
    
    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
