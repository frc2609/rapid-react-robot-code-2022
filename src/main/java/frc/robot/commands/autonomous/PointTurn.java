// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.utils.SimPID;

public class PointTurn extends CommandBase {
  /** Creates a new PointTurn. */
  SimPID pid;
  double angle;

  public PointTurn(double desiredAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    pid = new SimPID(4.9014, 0, 0.99603);
    angle = desiredAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_driveSubsystem.isDriveLocked = true;
    angle += RobotContainer.bodyNavx.getAngle();
    pid.setDesiredValue(angle);
    DriverStation.reportWarning("desired angle: " + angle, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calcPID(RobotContainer.bodyNavx.getAngle()) + 0.36185;
    RobotContainer.m_driveSubsystem.tankDriveVolts(
        -output, output);
    SmartDashboard.putNumber("getAngle", RobotContainer.bodyNavx.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_driveSubsystem.tankDriveVolts(0, 0);
    RobotContainer.m_driveSubsystem.isDriveLocked = false;
    DriverStation.reportWarning("end angle: " + angle, false);
    angle = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.bodyNavx.getAngle() - angle) < 2;
  }
}
