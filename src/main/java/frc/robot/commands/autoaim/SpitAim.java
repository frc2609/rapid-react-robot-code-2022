// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoaim;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SpitAim extends CommandBase {
  /** Creates a new ManualSpit. */
  double angle;
  double time,startTime;

  public SpitAim(double angle, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_shooterSubsystem.isSpitting = true;
    startTime = Timer.getFPGATimestamp();

    // RobotContainer.m_shooterSubsystem.enableAutoAim();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("SHOOT");
    RobotContainer.m_shooterSubsystem.isSpittingFullRotate = true;
    RobotContainer.m_shooterSubsystem.shootWithBackspin(2500);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Spitaim done");
    RobotContainer.m_shooterSubsystem.isSpittingFullRotate = false;
    RobotContainer.m_shooterSubsystem.isSpittingLowRotate = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(RobotContainer.m_shooterSubsystem.getRotatePos()-angle)<2;
    return (Timer.getFPGATimestamp()>=(startTime+time));
  }
}
