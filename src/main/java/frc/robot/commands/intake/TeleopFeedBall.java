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
import edu.wpi.first.wpilibj.DriverStation;

public class TeleopFeedBall extends CommandBase {
  private Intake m_intake;

  /** Creates a new TeleopFeedBall. */
  public TeleopFeedBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = RobotContainer.m_intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setUpperBelt(Constants.Motors.BELT_SPEED);
    DriverStation.reportError("RPMDelta" + (RobotContainer.m_shooterSubsystem.autoRPMsetp-RobotContainer.m_shooterSubsystem.getRPM()), false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setUpperBelt(1);
    m_intake.setLowerBelt(1);
    // AUTO STAGING
    // boolean isBallAtIntake = RobotContainer.m_shooterSubsystem.getIntakeSensor();
    // boolean isBallAtShooter = RobotContainer.m_shooterSubsystem.shooterSensor.get();

    // if(SmartDashboard.getBoolean(Constants.INTAKE_OVERRIDE_STRING, true)) {
    //   isBallAtIntake = false;
    //   isBallAtShooter = false;
    //   RobotContainer.operatorJoystick.setRumble(RumbleType.kLeftRumble, 1);
    // } else {
    //   RobotContainer.operatorJoystick.setRumble(RumbleType.kLeftRumble, 0);
    // }

    // if (isBallAtIntake == false && isBallAtShooter == false) {
    //   // No balls
    //   m_intake.setUpperBelt(Constants.Motors.BELT_SPEED);
    //   m_intake.setLowerBelt(Constants.Motors.BELT_SPEED);
    // } 
    // else if (isBallAtIntake == false && isBallAtShooter == true) {
    //   // Staged ball
    //   m_intake.setUpperBelt(Constants.Motors.BELT_SPEED);
    //   m_intake.setLowerBelt(Constants.Motors.BELT_SPEED);
    // } 
    // else if (isBallAtIntake == true && isBallAtShooter == false) {
    //   // Intake ball
    //   m_intake.setUpperBelt(Constants.Motors.BELT_SPEED);
    //   m_intake.setLowerBelt(Constants.Motors.BELT_SPEED);
    // }
    // else if (isBallAtIntake == true && isBallAtShooter == true) {
    //   // Full
    //   m_intake.setUpperBelt(Constants.Motors.BELT_SPEED);
    //   m_intake.setLowerBelt(-0.1);
    // }
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
    if(SmartDashboard.getBoolean(Constants.FEEDER_OVERRIDE_STRING, true)) {
      //RobotContainer.operatorJoystick.setRumble(RumbleType.kRightRumble, 1);
      return false;
    } else {
      RobotContainer.operatorJoystick.setRumble(RumbleType.kRightRumble, 0);
    }
    
    return !RobotContainer.m_shooterSubsystem.isTargetLocked();
  }
}