// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class TeleopIntakeBall extends CommandBase {
  private Intake m_intake;

  /** Creates a new TeleopFeedBall. */
  public TeleopIntakeBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = RobotContainer.m_intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setUpperBelt(Constants.Motors.BELT_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isBallAtIntake = RobotContainer.m_shooterSubsystem.getIntakeSensor();
    boolean isBallAtShooter = RobotContainer.m_shooterSubsystem.shooterSensor.get();

    if(SmartDashboard.getBoolean(Constants.INTAKE_OVERRIDE_STRING, true)) {
      isBallAtIntake = false;
      isBallAtShooter = false;
      RobotContainer.operatorJoystick.setRumble(RumbleType.kLeftRumble, 1);
    } else {
      RobotContainer.operatorJoystick.setRumble(RumbleType.kLeftRumble, 0);
    }

    if (isBallAtIntake == false && isBallAtShooter == false) {
      // No balls
      m_intake.setBelts(Constants.Motors.BELT_SPEED);
      m_intake.setIntakeBelt(Constants.Motors.INTAKE_SPEED);
    } 
    else if (isBallAtIntake == false && isBallAtShooter == true) {
      // Staged ball
      m_intake.setUpperBelt(-0.05);
      m_intake.setLowerBelt(Constants.Motors.BELT_SPEED);
      m_intake.setIntakeBelt(Constants.Motors.INTAKE_SPEED);

    } 
    else if (isBallAtIntake == true && isBallAtShooter == false) {
      // Intake ball
      m_intake.setBelts(Constants.Motors.BELT_SPEED);
      m_intake.setIntakeBelt(Constants.Motors.INTAKE_SPEED);

    }
    else if (isBallAtIntake == true && isBallAtShooter == true) {
      // Full
      m_intake.setBelts(0);
      m_intake.setIntakeBelt(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_intake.setBelts(0.0);
    // m_intake.setIntakeBelt(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}