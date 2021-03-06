package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Underglow extends SubsystemBase {
  // led controller pretends to be a PWM motor controller
  private final Spark LEDController = new Spark(Constants.LED.PWM_PORT);
  public static double color;

  public Underglow() {
    checkColor();
  }

  public void checkColor() {
    if (SmartDashboard.getBoolean("isRedTeam", true)) {
      color = Constants.LED.RED;
    } else {
      color = Constants.LED.BLUE;
    }
  }

  public void checkSweetSpot() {
    if (RobotContainer.m_shooterSubsystem.isSweetSpot()) {
      color = Constants.LED.GREEN;
    } else {
      color = Constants.LED.RED;
    }
  }

  public void setColor(double set) {
    color = set;
  }

  @Override
  public void periodic() {
    if(RobotContainer.m_shooterSubsystem.isAutoAim()){
      if(RobotContainer.m_shooterSubsystem.isTargetLocked()){
        color = Constants.LED.GREEN;
      }else{
        color = Constants.LED.RED;
      }
    }else{
    checkColor();
    color = Constants.LED.PURPLE;  // force set purple for now
    }
    LEDController.set(color); // changes colour of LED
  }
}