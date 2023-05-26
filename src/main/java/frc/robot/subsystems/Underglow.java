package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Underglow extends SubsystemBase {
  // REV Blinkin pretends to be a PWM motor controller
  private static Spark LEDController = new Spark(Constants.LED.PWM_PORT);
  public static double currentColor;

  public Underglow() {}

  public void setToAllianceColor() {
    if (SmartDashboard.getBoolean("isRedTeam", true)) {
      currentColor = Constants.LED.RED;
    } else {
      currentColor =  Constants.LED.BLUE;
    }
  }

  public void setPurple() {
    currentColor = Constants.LED.PURPLE;
  }

  @Override
  public void periodic() {
    LEDController.set(currentColor);
  }
}