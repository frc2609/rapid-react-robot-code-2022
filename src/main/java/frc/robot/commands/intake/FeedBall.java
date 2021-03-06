// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class FeedBall extends CommandBase {
  private Intake m_intake;
  private int outCounter = 0;
  double time, startTime;

  /** Creates a new Feed. */
  public FeedBall() {
    m_intake = RobotContainer.m_intakeSubsystem;
    this.time = Constants.AutoConstants.commandTimer;  // add another 2 seconds on top of constant value just in case
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
    if (RobotContainer.m_shooterSubsystem.isTargetLocked()) {
      m_intake.setUpperBelt(Constants.Motors.BELT_SPEED);
      m_intake.setLowerBelt(Constants.Motors.BELT_SPEED);
    }
    if (!RobotContainer.m_shooterSubsystem.stagingSensor.get()
        && !RobotContainer.m_shooterSubsystem.shooterSensor.get()
        && !RobotContainer.m_shooterSubsystem.getIntakeSensor()) {
      outCounter++;
    } else {
      outCounter = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setBelts(0.0);
    System.out.println("ending FeedBall");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return outCounter >= 5 || (Timer.getFPGATimestamp()>=(startTime+time));
  }
}
