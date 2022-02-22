// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

import java.lang.reflect.Array;

import com.revrobotics.ColorMatch;

public class ColorSensing extends SubsystemBase {
  public enum BallColor {
    red,
    blue,
    other,
  }

  private BallColor m_color;
  // targets
  private final Color kBlueTarget = new Color(0.200, 0.450, 0.300);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.332, 0.100);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  private final Color[] targetColors = { kBlueTarget, kGreenTarget, kRedTarget, kYellowTarget };
  // ports
  private final I2C.Port i2cPortRIO = I2C.Port.kOnboard;
  private final I2C.Port i2cPortMXP = I2C.Port.kMXP;
  // sensors
  private final ColorSensorV3 lowerColorSensor = new ColorSensorV3(i2cPortRIO);
  private final ColorSensorV3 upperColorSensor = new ColorSensorV3(i2cPortMXP);
  // private final ColorMatch colorMatcher = new ColorMatch();
  private int index = -1;

  /** Creates a new ColorSensing. */
  public ColorSensing() {
    /*
     * lowerColorSensor.configureColorSensor(ColorSensorV3.ColorSensorResolution.
     * kColorSensorRes18bit,
     * ColorSensorV3.ColorSensorMeasurementRate.kColorRate25ms,
     * ColorSensorV3.GainFactor.kGain3x);
     * 
     * upperColorSensor.configureColorSensor(ColorSensorV3.ColorSensorResolution.
     * kColorSensorRes18bit,
     * ColorSensorV3.ColorSensorMeasurementRate.kColorRate25ms,
     * ColorSensorV3.GainFactor.kGain3x);
     */
  }

  @Override
  public void periodic() {
    detectColor(lowerColorSensor, "Lower ");
    detectColor(upperColorSensor, "Upper ");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void detectColor(ColorSensorV3 colorSensor, String name) {
    // Detects the colour, outputs it into the dashboard, and sets the ball colour
    // ("ballIsBlue", "ballIsRed")
    String colorString;
    // FYI this ignores confidence and will always give a color
    Color match = match(colorSensor, targetColors).color;

    if (match == kBlueTarget) {
      colorString = "Blue";
      m_color = BallColor.blue;
    } else if (match == kRedTarget) {
      colorString = "Red";
      m_color = BallColor.red;
    } else if (match == kGreenTarget) {
      colorString = "Green";
      m_color = BallColor.other;
    } else if (match == kYellowTarget) {
      colorString = "Yellow";
      m_color = BallColor.other;
    } else {
      colorString = "Unknown";
      m_color = BallColor.other;
    }

    SmartDashboard.putNumber(name + "Red", colorSensor.getRed());
    SmartDashboard.putNumber(name + "Green", colorSensor.getGreen());
    SmartDashboard.putNumber(name + "Blue", colorSensor.getBlue());
    SmartDashboard.putBoolean(name + "isConnected", colorSensor.isConnected());
    // SmartDashboard.putNumber(name + "Confidence", match.confidence); // won't
    // work without using Rev code, uneccessary anyways, could implement it though
    SmartDashboard.putString(name + "Detected Color", colorString);
  }

  /*
   * Returns the color of the ball. Use friendlyBall to check whether ball
   * color matches team color or not.
   */
  public BallColor color() {
    return m_color;
  }

  /* Returns true if the ball color matches the alliance color. */
  public boolean friendlyBall() {
    // check if ball color matches color of current alliance
    boolean team = SmartDashboard.getBoolean("IsRedAlliance", true);
    switch (m_color) {
      case red:
        return team; // if team red, true, if team blue, false
      case blue:
        return !team; // if team red, false, if team blue, true
      default:
        return false; // otherwise return false
    }
  }

  private ColorMatchResult match(ColorSensorV3 colorSensor, Color[] targetColors) {
    /*
     * double distance;
     * Color match;
     * double shortestDistance = Double.MAX_VALUE;
     * double distance;
     * 
     * double currentR = (double) colorSensor.getRed();
     * double currentB = (double) colorSensor.getBlue();
     * double currentG = (double) colorSensor.getGreen();
     * 
     * double redRDifference = currentR - targetColors[2].red;
     * double redBDifference = currentB - targetColors[2].blue;
     * double
     * 
     * distance =
     */
    Color detectedColor = new Color(colorSensor.getRed(), colorSensor.getGreen(), colorSensor.getBlue());
    double magnitude = colorSensor.getRed() + colorSensor.getBlue() + colorSensor.getGreen();
    if (magnitude > 0) {
      double minDistance = 1.0;
      int idx = 0;

      for (int i = 0; i < targetColors.length; i++) {
        double targetDistance = CalculateDistance(targetColors[i], detectedColor);

        if (targetDistance < minDistance) {
          minDistance = targetDistance;
          idx = i;
        }
      }
      ColorMatchResult match = new ColorMatchResult(targetColors[idx], 1.0 - (minDistance / magnitude));
      return match;
    } else {
      return new ColorMatchResult(Color.kBlack, 0.0);
    }
  }

  private static double CalculateDistance(Color color1, Color color2) {
    double redDiff = color1.red - color2.red;
    double greenDiff = color1.green - color2.green;
    double blueDiff = color1.blue - color2.blue;

    return Math.sqrt((redDiff * redDiff + greenDiff * greenDiff + blueDiff * blueDiff) / 2);
  }
}
