package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import java.lang.Math;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterLeftMotor = new CANSparkMax(Constants.SHOOTER_LEFT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax shooterRightMotor = new CANSparkMax(Constants.SHOOTER_RIGHT_MOTOR, MotorType.kBrushless);
  private Joystick m_stick;

  public Shooter(Joystick stick)
  {
    m_stick = stick;
  }

  @Override
  public void periodic()
  {
    double move = m_stick.getRawAxis(Constants.RIGHT_TRIGGER_AXIS);
    setMotors(Math.abs(move) < Constants.JOYSTICK_DRIFT_TOLERANCE ? 0 : move);
  }

  public void setMotors(double set)
  {
    SmartDashboard.putNumber("Shooter Speed", set);
    shooterLeftMotor.set(-set);
    shooterRightMotor.set(set);
  }

}
