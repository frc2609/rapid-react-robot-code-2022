// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeBallAlways extends CommandBase {
  /** Creates a new IntakeBallAlways. */
  private Intake m_intake;

  public IntakeBallAlways() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = RobotContainer.m_intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeBelt(Constants.Motors.INTAKE_SPEED);
    m_intake.setLowerBelt(Constants.Motors.BELT_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
