// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class AutoAim extends CommandBase {
  /** Creates a new AutoAim. */
  private Shooter m_shooter;
  private Joystick m_stick;

  public AutoAim() {
    m_shooter = RobotContainer.m_shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.enableAutoAim();
    m_shooter.isFlywheelDisabled = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_shooter.disableAutoAim();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_shooterSubsystem.isTargetLocked();
  }
}
