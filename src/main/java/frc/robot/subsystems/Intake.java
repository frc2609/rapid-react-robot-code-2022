package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants;


public class Intake extends SubsystemBase{

  private final PWMVictorSPX lowerBeltMotor = new PWMVictorSPX(Constants.LOWER_BELT_MOTOR);
  private final PWMVictorSPX upperBeltMotor = new PWMVictorSPX(Constants.UPPER_BELT_MOTOR);
  private final PWMVictorSPX intakeLiftMotor = new PWMVictorSPX(Constants.INTAKE_LIFT_MOTOR);
  private final PWMVictorSPX intakeBallMotor = new PWMVictorSPX(Constants.INTAKE_BALL_MOTOR);
  private final Joystick m_stick;

  public Intake(Joystick stick) 
  {
    m_stick = stick;
  }

  @Override
  public void periodic() {
    if (m_stick.getRawButton(Constants.B_BUTTON)) {
      setBelts(1);
    }
    else { 
      setBelts(0); 
    }
    
    if (m_stick.getRawButton(Constants.A_BUTTON)) {
      setShooter(0.5);
    }
    else {
      setShooter(0);
    }

    intakeLiftMotor.set(m_stick.getRawAxis(Constants.RIGHT_STICK_X_AXIS) * 0.25);
  }

  @Override
  public void simulationPeriodic() {}

  public void setBelts(double speed) {
    lowerBeltMotor.set(speed);
    upperBeltMotor.set(speed);
  }

  public void setShooter(double speed) {
    intakeBallMotor.set(speed);
  }
}
