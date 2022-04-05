// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class SetIntakeBelt extends InstantCommand {
  /** Creates a new SetIntakeBelt. */
  double speed;

  public SetIntakeBelt(double speed) {
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_intakeSubsystem.setIntakeBelt(speed);
  }

}
