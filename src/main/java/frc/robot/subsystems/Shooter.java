package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

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
    setMotors(m_stick.getRawAxis(Constants.RIGHT_STICK_X_AXIS));
  }

  public void setMotors(double set)
  {
    shooterLeftMotor.set(set);
    shooterRightMotor.set(-set);
  }

}
