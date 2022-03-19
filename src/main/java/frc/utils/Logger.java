// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.io.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class Logger {

  private BufferedWriter writer;
  private boolean logging = true;
  private final String loggerBoolean = "Logging";
  private static Logger instance;
  private String fileName = "beaverlog";
  private final String SDFileName = "File Name: ";
  DriverStation ds;

  private int max = 0;
  private Climber climber;
  private String path;

  public static Logger getInstance() {
    if (instance == null) {
      instance = new Logger();
    }
    return instance;
  }

  private Logger() {
    this.ds = DriverStation.getInstance();
    SmartDashboard.putBoolean(this.loggerBoolean, this.logging);
    // this.logging= SmartDashboard.getBoolean(this.loggerBoolean);

    SmartDashboard.putString(this.SDFileName, this.fileName);
    this.fileName = SmartDashboard.getString(SDFileName, "beaverlog");
    File f = new File("/home/lvuser/beaverlogs");
    if (!f.exists()) {
      f.mkdir();
    }

    File[] files = new File("/home/lvuser/beaverlogs").listFiles();
    if (files != null) {
      for (File file : files) {
        if (file.isFile()) {
          // System.out.println(file.getName());
          try {
            int index = Integer.parseInt(file.getName().split("_")[0]);
            if (index > max) {
              max = index;
            }
          } catch (Exception e) {
            e.printStackTrace();
          }
        }
      }
    } else {
      max = 0;
    }
  }

  public void setClimber(Climber climber) {
    this.climber = climber;
  }

  public void openFile() {
    if (this.wantToLog() || this.ds.isFMSAttached()) {
      try {
        path = this.getPath();
        this.writer = new BufferedWriter(new FileWriter(path));
        this.writer.write(
            "FPGATime, encLeft, encRight, velLeft, velRight, leftCurrent, rightCurrent, leftVoltage, rightVoltage, leftEncSetp, rightEncSetp, leftVelSetp, rightVelSetp, leftAccelSetp, rightAccelSetp, gyroSetp, yaw, angle, ");
        // this.writer.write(String.format("%.3f", (double)
        // MPConstants.cruiseVelocity));
        // this.writer.write(" ,");
        // this.writer.write(String.format("%.3f", (double) MPConstants.kP));
        // this.writer.write(" ,");
        // this.writer.write(String.format("%.3f", (double) MPConstants.kI));
        // this.writer.write(" ,");
        // this.writer.write(String.format("%.3f", (double) MPConstants.kD));
        // this.writer.write(" ,");
        // this.writer.write(String.format("%.3f", (double) MPConstants.kV));
        // this.writer.write(" ,");
        // this.writer.write(String.format("%.3f", (double) MPConstants.kA));
        this.writer.newLine();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  public void openFileTele() {
    if (this.wantToLog() || this.ds.isFMSAttached()) {
      try {
        path = this.getPath();
        this.writer = new BufferedWriter(new FileWriter(path));
        this.writer.write(
            "FPGATime, shooterL, shooterR, shooterVoltageL, shooterVoltageR, vaultboyL, vaultboyR, vaultboyVoltageL, vaultboyVoltageR");
        this.writer.newLine();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  private String getPath() {
    this.fileName = SmartDashboard.getString(SDFileName, "beaverlog");
    if (this.ds.isFMSAttached()) {
      return String.format("/home/lvuser/beaverlogs/%d_%s_%d_log.csv", ++this.max, this.ds.getAlliance().name(),
          this.ds.getLocation());
    } else if (this.fileName != null) {
      return String.format("/home/lvuser/beaverlogs/%d_%s.csv", ++this.max, this.fileName);
    } else {
      return String.format("/home/lvuser/beaverlogs/%d_log.csv", ++this.max);
    }
  }

  public void logTele() {
    if (this.wantToLog()) {
      try {
        // int ,%d
        // double ,%.3f
        this.writer.write(String.format("%.3f", Timer.getFPGATimestamp()));
        // this.writer.write(String.format(",%d", new java.util.Date().getTime()));
        this.writer.write(String.format(",%.3f", climber.getHookPosition()));
        this.writer.write(String.format(",%.3f", climber.getArmKinematics().getArmAngle()));
        this.writer.write(String.format(",%.3f", climber.getArmKinematics().getArmVelocity()));
        this.writer.write(String.format(",%.3f", climber.getArmPosition()));
        this.writer.write(String.format(",%.3f", climber.getArmKinematics().getBodyAngle()));
        this.writer.write(String.format(",%.3f", climber.getArmKinematics().getBodyVelocity()));
        this.writer.write(String.format(",%.3f", climber.barMotor.getAppliedOutput()));
        this.writer.write(String.format(",%.3f", climber.hookMotor.getAppliedOutput()));
        this.writer.write(String.format(",%.3f", climber.barMotor.getOutputCurrent()));
        this.writer.write(String.format(",%.3f", climber.hookMotor.getOutputCurrent()));
        this.writer.write(String.format(",%b", climber.getArmKinematics().isHooked()));

        this.writer.newLine();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  public void logAll() {
    if (this.wantToLog()) {
      try {
        // int ,%d
        // double ,%.3f
        this.writer.write(String.format("%.3f", Timer.getFPGATimestamp()));
        // this.writer.write(String.format(",%d", new java.util.Date().getTime()));

        this.writer.newLine();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  public void logMM() {
    if (this.wantToLog()) {
      try {
        // int ,%d
        // double ,%.3f
        this.writer.write(String.format("%.3f", Timer.getFPGATimestamp()));
        // this.writer.write(String.format(",%d", new java.util.Date().getTime()));

        this.writer.newLine();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  public void logInterpolatedMP(double[] leftInterpolated, double[] rightInterpolated) {
    try {
      // int ,%d
      // double ,%.3f
      this.writer.write(String.format("%.3f", Timer.getFPGATimestamp()));
      // this.writer.write(String.format(",%d", new java.util.Date().getTime()));

      this.writer.newLine();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public boolean wantToLog() {
    this.logging = SmartDashboard.getBoolean(this.loggerBoolean, true);
    return this.logging;
  }

  public void close() {
    if (this.wantToLog()) {
      if (this.writer != null) {
        try {
          this.writer.close();
        } catch (IOException e) {
          e.printStackTrace();
        }
      }
    }
  }
}