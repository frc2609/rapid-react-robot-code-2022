// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class ShootStagedBall extends CommandBase {
  private int outCounter = 0;
  double time, startTime;

  /** Creates a new Feed. */
  public ShootStagedBall() {
    this.time = 2;  // end after 0.5
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_intake.setUpperBelt(Constants.Motors.BELT_SPEED);
    // m_intake.setLowerBelt(Constants.Motors.BELT_SPEED);
    startTime = Timer.getFPGATimestamp();
    System.out.println("initializing FeedBall");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_intakeSubsystem.setBelts(Constants.Motors.BELT_SPEED);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intakeSubsystem.setBelts(0.0);
    RobotContainer.m_shooterSubsystem.setFlywheelAndHoodRpmPower(0);
    RobotContainer.m_shooterSubsystem.isSpitting = false;
    RobotContainer.m_shooterSubsystem.isSpittingLowRotate = false;
    RobotContainer.m_shooterSubsystem.isSpittingFullRotate = false;
    System.out.println("ending ShootStagedBall");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp()>=(startTime+time));
  }
}
