// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MP;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class RamseteFactory {
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.DriveKin.ksVolts,
        Constants.DriveKin.kvVoltSecondsPerMeter,
        Constants.DriveKin.kaVoltSecondsSquaredPerMeter),
        Constants.DriveKin.kDriveKinematics,
        10);


TrajectoryConfig config_back =
new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                     Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(Constants.DriveKin.kDriveKinematics)
    // Apply the voltage constraint
    .addConstraint(autoVoltageConstraint).setReversed(true);

TrajectoryConfig config =
    new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
// Add kinematics to ensure max speed is actually obeyed
.setKinematics(Constants.DriveKin.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint).setReversed(false);

// An example trajectory to follow.  All units in meters.
Trajectory startToBall = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0.0, 0.0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
      // new Translation2d(1, 1),
      // new Translation2d(2, 0.5)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(0.8, 0, new Rotation2d(-0)),
    // Pass config
    config
);

    

private static RamseteFactory m_instance;

private RamseteFactory() {
}
public static RamseteFactory getInstance() {
    if (m_instance == null) {
        m_instance = new RamseteFactory();
    }
    return m_instance;
}
public RamseteCommand startToBallCommand = new RamseteCommand(startToBall, RobotContainer.m_driveSubsystem::getPose, new RamseteController(Constants.DriveKin.kRamseteB, Constants.DriveKin.kRamseteZeta),
     new SimpleMotorFeedforward(
        Constants.DriveKin.ksVolts,
        Constants.DriveKin.kvVoltSecondsPerMeter,
        Constants.DriveKin.kaVoltSecondsSquaredPerMeter), Constants.DriveKin.kDriveKinematics, RobotContainer.m_driveSubsystem::getWheelSpeeds,
        new PIDController(Constants.DriveKin.kPDriveVel, 0, 0) , new PIDController(Constants.DriveKin.kPDriveVel, 0, 0), RobotContainer.m_driveSubsystem::tankDriveVolts, RobotContainer.m_driveSubsystem);
}
