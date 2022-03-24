// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.utils.SimPID;

public class LockDrive extends CommandBase {
  private SimPID rightDrivePID;
  private SimPID leftDrivePID;
  private SimPID headingPID;

  /** Creates a new LockDrive. */

  public LockDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    rightDrivePID = new SimPID(0.016, 0, 0.3);
    leftDrivePID = new SimPID(0.016, 0, 0.3);
    headingPID = new SimPID(0.016, 0, 0.3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_driveSubsystem.isDriveLocked = true;
    rightDrivePID.setDesiredValue(RobotContainer.m_driveSubsystem.getRightMotorPosition());
    leftDrivePID.setDesiredValue(RobotContainer.m_driveSubsystem.getLeftMotorPosition());
    headingPID.setDesiredValue(RobotContainer.bodyNavx.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_driveSubsystem.setMotors(
      leftDrivePID.calcPID(RobotContainer.m_driveSubsystem.getLeftMotorPosition()) - headingPID.calcPID(RobotContainer.bodyNavx.getAngle()),
      rightDrivePID.calcPID(RobotContainer.m_driveSubsystem.getRightMotorPosition()) + headingPID.calcPID(RobotContainer.bodyNavx.getAngle())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driveSubsystem.isDriveLocked = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
