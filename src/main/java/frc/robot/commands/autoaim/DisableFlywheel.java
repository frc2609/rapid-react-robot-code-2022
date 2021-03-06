// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoaim;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/* 
* Disables flywheel, allows auto to keep autoaim running so that it doesn't
* lose track of the target while saving power by not running the flywheel.
*/
public class DisableFlywheel extends InstantCommand {
  public DisableFlywheel() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("initializing DisableFlywheel");

    RobotContainer.m_shooterSubsystem.disableFlywheel();
  }
}
