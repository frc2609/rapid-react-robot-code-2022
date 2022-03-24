// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class TeleopFeedBall extends CommandBase {
  private Intake m_intake;

  /** Creates a new TeleopFeedBall. */
  public TeleopFeedBall() {
    m_intake = RobotContainer.m_intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setUpperBelt(Constants.Motors.BELT_SPEED);
    m_intake.setLowerBelt(Constants.Motors.BELT_SPEED);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setUpperBelt(0);
    m_intake.setLowerBelt(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.m_shooterSubsystem.isTargetLocked();
  }
}
