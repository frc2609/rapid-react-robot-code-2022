package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Underglow extends SubsystemBase {
  // led controller pretends to be a PWM motor controller
  private final Spark LEDController = new Spark(Constants.LED.PWM_PORT);
  public static double color;

  public Underglow() {
    checkColor();
    SmartDashboard.putNumber("LED Controller Colour", color);
  }

  public void checkColor() {
    if (SmartDashboard.getBoolean("isRedTeam", true)) {
      color = Constants.LED.RED;
    } else {
      color = Constants.LED.BLUE;
    }
  }

  public void setColor(double set) {
    color = set;
  }

  @Override
  public void periodic() {
    // color = SmartDashboard.getNumber("LED Controller Colour", color);
    checkColor();
    LEDController.set(color); // changes colour of LED
    SmartDashboard.putNumber("LED Controller Colour", color);
  }
}