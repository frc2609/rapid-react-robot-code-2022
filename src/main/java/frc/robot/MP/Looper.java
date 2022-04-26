// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MP;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List
 * object. They are started when the robot powers up and stopped after the
 * match.
 */
public class Looper {
  public final double kPeriod = 0.01; // 100hz loop

  private boolean running_;

  private final Notifier notifier_;
  private final List<Loop> loops_;
  private final Object taskRunningLock_ = new Object();
  private double timestamp_ = 0;
  private double dt_ = 0;
  private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() {
    @Override
    public void runCrashTracked() {
      synchronized (taskRunningLock_) {
        if (running_) {
          double now = Timer.getFPGATimestamp();
          for (Loop loop : loops_) {
            loop.onLoop();
          }
          dt_ = now - timestamp_;
          timestamp_ = now;
          // outputToSmartDashboard();
        }
      }
    }
  };

  public Looper() {
    notifier_ = new Notifier(runnable_);
    running_ = false;
    loops_ = new ArrayList<>();
  }

  public synchronized void register(Loop loop) {
    synchronized (taskRunningLock_) {
      loops_.add(loop);
    }
  }

  public synchronized void start() {
    if (!running_) {
      System.out.println("Starting loops");
      synchronized (taskRunningLock_) {
        timestamp_ = Timer.getFPGATimestamp();
        for (Loop loop : loops_) {
          loop.onStart();
        }
        running_ = true;
      }
      notifier_.startPeriodic(kPeriod);
    }
  }

  public synchronized void stop() {
    if (running_) {
      System.out.println("Stopping loops");
      notifier_.stop();
      synchronized (taskRunningLock_) {
        running_ = false;
        for (Loop loop : loops_) {
          System.out.println("Stopping " + loop);
          loop.onStop();
        }
      }
    }
  }

  public void outputToSmartDashboard() {
    // SmartDashboard.putNumber("looper_dt", dt_);
  }
}