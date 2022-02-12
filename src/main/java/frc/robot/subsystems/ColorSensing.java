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
import com.revrobotics.ColorMatch;

public class ColorSensing extends SubsystemBase {
  public enum BallColor
  {
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
  // ports
  private final I2C.Port i2cPortRIO = I2C.Port.kOnboard;
  private final I2C.Port i2cPortMXP = I2C.Port.kMXP;
  // sensors
  private final ColorSensorV3 lowerColorSensor = new ColorSensorV3(i2cPortRIO);
  private final ColorSensorV3 upperColorSensor = new ColorSensorV3(i2cPortMXP);
  private final ColorMatch colorMatcher = new ColorMatch();
  
  /** Creates a new ColorSensing. */
  public ColorSensing() {}

  @Override
  public void periodic() {
    detectColor(lowerColorSensor, "Lower ");
    detectColor(upperColorSensor, "Upper ");
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void detectColor(ColorSensorV3 colorSensor, String name) 
  {
    // Detects the colour, outputs it into the dashboard, and sets the ball colour ("ballIsBlue", "ballIsRed")
    Color detectedColor = colorSensor.getColor();
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
      m_color = BallColor.blue;
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      m_color = BallColor.red;
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      m_color = BallColor.other;
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      m_color = BallColor.other;
    } else {
      colorString = "Unknown";
      m_color = BallColor.other;
    }

    SmartDashboard.putNumber(name + "Red", detectedColor.red);
    SmartDashboard.putNumber(name + "Green", detectedColor.green);
    SmartDashboard.putNumber(name + "Blue", detectedColor.blue);
    SmartDashboard.putNumber(name + "Confidence", match.confidence);
    SmartDashboard.putString(name + "Detected Color", colorString);
  }
  /* Returns the color of the ball. Use friendlyBall to check whether ball
  *  color matches team color or not. */
  public BallColor color() { return m_color; }

  /* Returns true if the ball color matches the alliance color. */
  public boolean friendlyBall()
  {
    // check if ball color matches color of current alliance
    boolean team = SmartDashboard.getBoolean("IsRedAlliance", true);
    switch (m_color)
    {
      case red:
      return team; // if team red, true, if team blue, false
      case blue:
      return !team; // if team red, false, if team blue, true
      default:
      return false; // otherwise return false
    }
  }
}

