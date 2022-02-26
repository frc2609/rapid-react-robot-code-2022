package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import java.lang.Math; // for Math.abs()

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterLeftMotor = new CANSparkMax(Constants.SHOOTER_LEFT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax shooterRightMotor = new CANSparkMax(Constants.SHOOTER_RIGHT_MOTOR, MotorType.kBrushless);
  private Joystick m_stick;
  private boolean m_pressed = false;
  private double m_speed = 0; // double to avoid integer division

  public Shooter(Joystick stick)
  {
    m_stick = stick;
    SmartDashboard.putNumber("Shooter Set", 0);
  }

  @Override
  public void periodic()
  {
    //double move = m_stick.getRawAxis(Constants.RIGHT_TRIGGER_AXIS);
    //setMotors(Math.abs(move) < Constants.JOYSTICK_DRIFT_TOLERANCE ? 0 : move);
    int pov = m_stick.getPOV();
    SmartDashboard.putNumber("Joystick POV", pov);
    if (!m_pressed)
    {
      switch (pov)
      {
        case Constants.POV_UP_BUTTON:
        m_pressed = true;
        m_speed += 1;
        break;
        case Constants.POV_DOWN_BUTTON:
        m_pressed = true;
        m_speed -= 1;
        break;
        case Constants.POV_LEFT_BUTTON:
        m_pressed = true;
        m_speed -= 10;
        break;
        case Constants.POV_RIGHT_BUTTON:
        m_pressed = true;
        m_speed += 10;
        break;
      }
    }
    if (pov == -1) m_pressed = false;
    SmartDashboard.putNumber("Shooter Set", m_speed);
    setMotors(m_speed / 100);
  }

  public void setMotors(double set)
  {
    SmartDashboard.putNumber("Shooter Speed", set);
    shooterLeftMotor.set(-set);
    shooterRightMotor.set(set);
  }

}
