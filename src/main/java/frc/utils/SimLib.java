// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;
/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author Mike DiRamio
 */
public class SimLib {

  public static double limitValue(double val) {
    return SimLib.limitValue(val, 1.0);
  }

  public static double limitValue(double val, double max) {
    if (val > max) {
      return max;
    } else if (val < -max) {
      return -max;
    } else {
      return val;
    }
  }

  public static double limitValue(double val, double max, double min) {
    if (val > max) {
      return max;
    } else if (val < min) {
      return min;
    } else {
      return val;
    }
  }

  public static double squareMaintainSign(double val) {
    double output = val * val;

    // was originally negative
    if (val < 0) {
      output = -output;
    }

    return output;
  }

  public static double power3MaintainSign(double val) {
    double output = val * val * val;
    return output;
  }

  public static double calcLeftTankDrive(double x, double y) {
    return SimLib.limitValue(y + x);
  }

  public static double calcRightTankDrive(double x, double y) {
    return SimLib.limitValue(y - x);
  }

  public static double max(double a, double b, double c) {
    a = Math.abs(a);
    b = Math.abs(b);
    c = Math.abs(c);
    if (a > b && a > c) {
      return a;
    } else if (b > c) {
      return b;
    } else {
      return c;
    }
  }
}
