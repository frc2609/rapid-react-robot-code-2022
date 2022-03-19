// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MP;

/**
 * Runnable class with reports all uncaught throws to CrashTracker
 */
public abstract class CrashTrackingRunnable implements Runnable {

  @Override
  public final void run() {
    try {
      runCrashTracked();
    } catch (Throwable t) {
      // CrashTracker.logThrowableCrash(t);
      System.out.println(t.getMessage());
      System.out.println(t.getStackTrace());
      throw t;
    }
  }

  public abstract void runCrashTracked();
}