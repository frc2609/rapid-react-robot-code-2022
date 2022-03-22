// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/** Add your docs here. */
public class RamseteFactory {
    public static final double ksVolts = 0.20434;
    public static final double kvVoltSecondsPerMeter = 0.85417;
    public static final double kaVoltSecondsSquaredPerMeter = 0.10137;

    public static final double kPDriveVel = 1.0742;
    public static final double kTrackwidthFeet = (21.75*12);

    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthFeet);

    public static final double kMaxSpeedFeetPerSecond = 12;
    public static final double kMaxAccelerationFeetPerSecondSquared = 12;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
