// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.FiniteDoubleQueue;

/** Add your docs here. */
// I'm going to assume this is for logging so it's gonna go eventually
@Deprecated
public class ArmKinematics {
  private AHRS armNavx, bodyNavx;
  private FiniteDoubleQueue armAvg, bodyAvg;

  public ArmKinematics(AHRS bodyAhrs) {
    try {
      armNavx = new AHRS(SerialPort.Port.kUSB);
    } catch (RuntimeException ex) {
      DriverStation.reportError("armNavx failed to connect", true);
    }
    bodyNavx = bodyAhrs;
    // try {
    //   bodyNavx = new AHRS(SerialPort.Port.kMXP);
    // } catch (RuntimeException ex) {
    //   DriverStation.reportError("bodyNavx failed to connect", true);
    // }
    armAvg = new FiniteDoubleQueue(5);
    bodyAvg = new FiniteDoubleQueue(5);
  }

  public double getArmAngle() {
    return armNavx.getYaw();
  }

  public double getBodyAngle() {
    return bodyNavx.getYaw();
  }

  public double getArmVelocity() {
    return armNavx.getRate();
  }

  public double getBodyVelocity() {
    return bodyNavx.getRate();
  }

  public void update() {
    armAvg.push(getArmVelocity());
    bodyAvg.push(getBodyVelocity());
    putSD();
  }

  public double getArmAverage() {
    return armAvg.getAverage();
  }

  public double getBodyAverage() {
    return bodyAvg.getAverage();
  }

  public boolean isHooked() {
    if (Math.abs(getArmAverage()) < Math.abs(getBodyAverage())) {
      return true;
    } else {
      return false;
    }
    // if (getArmAverage() == 0) {
    // if (Math.signum(getBodyAverage()) > 0) { // assume positive means body
    // swinging back
    // return true;
    // } else {
    // DriverStation.reportWarning("Arm stationary, body swinging forward", false);
    // return false;
    // }
    // } else {
    // if (Math.signum(getArmAverage()) > 0) {
    // // arm moving up
    // DriverStation.reportWarning("Arm moving up", false);
    // return false;
    // } else if (Math.signum(getBodyAverage()) > 0) { // assume positive means body
    // swinging back
    // // check if arm is near stationary?
    // return true;
    // } else {
    // DriverStation.reportError("Hooking detection inconclusive", false);
    // return false;
    // }
    // }
  }

  public void putSD() {

    // SmartDashboard.putNumber("getArmAngle()", getArmAngle());
    // SmartDashboard.putNumber("getBodyAngle()", getBodyAngle());
    // SmartDashboard.putNumber("getArmAverage()", getArmAverage());
    // SmartDashboard.putNumber("getBodyAverage()", getBodyAverage());
    // SmartDashboard.putNumber("getArmVelocity()", getArmVelocity());
    // SmartDashboard.putNumber("getBodyVelocity()", getBodyVelocity());
    // SmartDashboard.putBoolean("isHooked()", isHooked());
  }
}
