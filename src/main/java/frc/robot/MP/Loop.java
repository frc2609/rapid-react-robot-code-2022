// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MP;

/**
 * Interface for loops, which are routine that run periodically in the robot
 * code (such as periodic gyroscope calibration, etc.)
 */
public interface Loop {
  public void onStart();

  public void onLoop();

  public void onStop();
}
