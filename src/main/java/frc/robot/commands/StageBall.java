// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class StageBall extends CommandBase {
  /** Creates a new StageBall. */
  public StageBall() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!RobotContainer.m_shooterSubsystem.stagingSensor.get() && RobotContainer.m_shooterSubsystem.getIntakeSensor()){
      RobotContainer.m_intakeSubsystem.setBelts(Constants.Motors.BELT_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intakeSubsystem.setBelts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.m_shooterSubsystem.stagingSensor.get();
  }
}
