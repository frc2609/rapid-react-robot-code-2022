// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MP;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

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



    

private static RamseteFactory m_instance;

public Map<String, Trajectory> trajectoryMap = new HashMap<String, Trajectory>();


Trajectory startToBall = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0.0, 0.0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
      // new Translation2d(1, 1),
      // new Translation2d(2, 0.5)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(-1, 0, new Rotation2d(-0)),
    // Pass config
    config_back);

Trajectory crossTaxi = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0.0, 0.0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
      // new Translation2d(1, 1),
      // new Translation2d(2, 0.5)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(-4, 0, new Rotation2d(-0)),
    // Pass config
    config_back);

Trajectory twoBallStartToBall = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0.0, 0.0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
      // new Translation2d(1, 1),
      // new Translation2d(2, 0.5)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(-0.9, 0, new Rotation2d(-0)),
    // Pass config
    config_back);

    

// ball at 2.6, 1.6

Trajectory firstBallToSecondSetup = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0.0, 0.0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
      // new Translation2d(1, 1),
      // new Translation2d(2, 0.5)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, -1.6, new Rotation2d(-0)),
    // Pass config
    config);

// 1.2, 0.4, 3deg

Trajectory secondSetupToBall = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0.0, 0.0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
      // new Translation2d(1, 1),
      // new Translation2d(2, 0.5)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(-1.5, -0.4, new Rotation2d(-30)),
    // Pass config
    config_back);

    // Trajectory thirdBallToForth = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0.0, 0.0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(
    //       // new Translation2d(1, 1),
    //       // new Translation2d(2, 0.5)
    //     ),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(-1.5, -0.4, new Rotation2d(0)),
    //     // Pass config
    //     config_back);
    
private RamseteFactory() {
    trajectoryMap.put("startToBall", startToBall);
    trajectoryMap.put("firstBallToSecondSetup", firstBallToSecondSetup);
    trajectoryMap.put("secondSetupToBall", secondSetupToBall);
    trajectoryMap.put("twoBallStartToBall", twoBallStartToBall);
    trajectoryMap.put("crossTaxi", twoBallStartToBall);
    
    
}
public Trajectory getTrajectory(String pathName){
    return trajectoryMap.get(pathName);
}
public void printPath(){
    
    System.out.println(startToBall.toString());
    System.out.println(startToBall.getTotalTimeSeconds());
}
    
public static RamseteFactory getInstance() {
    if (m_instance == null) {
        m_instance = new RamseteFactory();
        System.out.println("initing");
    }
    return m_instance;
}


public RamseteCommand constructRamseteCommand(String pathName){
    return new RamseteCommand(getTrajectory(pathName), 
    RobotContainer.m_driveSubsystem::getPose, 
    new RamseteController(Constants.DriveKin.kRamseteB, Constants.DriveKin.kRamseteZeta),
    new SimpleMotorFeedforward(
       Constants.DriveKin.ksVolts,
       Constants.DriveKin.kvVoltSecondsPerMeter,
       Constants.DriveKin.kaVoltSecondsSquaredPerMeter),
        Constants.DriveKin.kDriveKinematics, 
        RobotContainer.m_driveSubsystem::getWheelSpeeds,
       new PIDController(Constants.DriveKin.kPDriveVel, 0, 0) ,
        new PIDController(Constants.DriveKin.kPDriveVel, 0, 0),
         RobotContainer.m_driveSubsystem::tankDriveVolts,
          RobotContainer.m_driveSubsystem);
}
}


