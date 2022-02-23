// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;

public class ColorSensing extends SubsystemBase {
  public enum BallColor {
    red,
    blue,
    no_ball,
  }
  private BallColor m_color;
  // ports
  private final I2C.Port i2cPortRIO = I2C.Port.kOnboard;
  private final I2C.Port i2cPortMXP = I2C.Port.kMXP;
  // sensors
  private final ColorSensorV3 lowerColorSensor = new ColorSensorV3(i2cPortRIO);
  private final ColorSensorV3 upperColorSensor = new ColorSensorV3(i2cPortMXP);

  /** Creates a new ColorSensing. */
  public ColorSensing() {}

  @Override
  public void periodic() {
    printBallColor(lowerColorSensor, "Lower ");
    printBallColor(upperColorSensor, "Upper ");
  }

  @Override
  public void simulationPeriodic() {}

  private void printBallColor(ColorSensorV3 colorSensor, String name) {
    checkColor(colorSensor);
    SmartDashboard.putBoolean(name + "Friendly Ball ", friendlyBall());
    SmartDashboard.putNumber(name + "Proximity", colorSensor.getProximity());
    SmartDashboard.putBoolean(name + "RED BALL", ballIsRed(colorSensor));
    SmartDashboard.putBoolean(name + "BALL IS THERE", ballIsThere(colorSensor));
  }

  private void printDebugInfo(ColorSensorV3 colorSensor, String name) {
    SmartDashboard.putNumber(name + "Red", colorSensor.getRed());
    SmartDashboard.putNumber(name + "Green", colorSensor.getGreen());
    SmartDashboard.putNumber(name + "Blue", colorSensor.getBlue() * 2);
    SmartDashboard.putNumber(name + "Proximity", colorSensor.getProximity());
    SmartDashboard.putBoolean(name + "isConnected", colorSensor.isConnected());
  }

  private void checkColor(ColorSensorV3 colorSensor) {
    if (ballIsThere(colorSensor)) {
      if (ballIsRed(colorSensor)) {
        m_color = BallColor.red;
      } else {
        m_color = BallColor.blue;
      }
    } else {
      m_color = BallColor.no_ball;
    }
  }
  
  private boolean ballIsThere(ColorSensorV3 colorSensor) {
    // this is NOT an actual unit
    int threshold = 400;
    return colorSensor.getProximity() > threshold;
  }

  // does NOT guarantee there is a ball there (use redBall and blueBall)
  private boolean ballIsRed(ColorSensorV3 colorSensor) {
    Color color = colorSensor.getColor();
    double red = color.red;
    double blue = color.blue;
    return red > blue;
  }

  /* Returns true if a ball is present and it is red */
  public boolean redBall(ColorSensorV3 colorSensor) {
    return ballIsRed(colorSensor) && ballIsThere(colorSensor);
  }

  /* Returns true if a ball is present and it is blue */
  public boolean blueBall(ColorSensorV3 colorSensor) {
    return !ballIsRed(colorSensor) && ballIsThere(colorSensor);
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

  /* Returns color of the ball */
  public BallColor color() {
    return m_color;
  }
}
