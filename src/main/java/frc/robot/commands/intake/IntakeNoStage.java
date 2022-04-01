// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeNoStage extends CommandBase {
  private Intake m_intake;
  private int whereToIntake = 2; // 1 - stage, 2 - intake, 0 - intake needs staging

  public IntakeNoStage() {
    m_intake = RobotContainer.m_intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("initializing IntakeNoStage");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (whereToIntake == 2 && !RobotContainer.m_shooterSubsystem.getIntakeSensor()) {
      m_intake.setIntakeBelt(Constants.Motors.INTAKE_SPEED);
      m_intake.setLowerBelt(Constants.Motors.BELT_SPEED * 0.5);
    } else if (whereToIntake == 2 && RobotContainer.m_shooterSubsystem.getIntakeSensor()) {
      m_intake.setBelts(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ending IntakeNoStage");

    m_intake.setIntakeBelt(0.0);
    m_intake.setBelts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_shooterSubsystem.getIntakeSensor();

  }
}
