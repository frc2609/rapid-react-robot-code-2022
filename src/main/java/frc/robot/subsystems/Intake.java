package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.XboxConstants;
import frc.robot.Constants.PwmMotorIdConstants;


public class Intake extends SubsystemBase{

  private final PWMVictorSPX lowerBeltMotor = new PWMVictorSPX(PwmMotorIdConstants.LOWER_BELT_MOTOR);
  private final PWMVictorSPX upperBeltMotor = new PWMVictorSPX(PwmMotorIdConstants.UPPER_BELT_MOTOR);
  private final PWMVictorSPX intakeLiftMotor = new PWMVictorSPX(PwmMotorIdConstants.INTAKE_LIFT_MOTOR);
  private final PWMVictorSPX intakeBallMotor = new PWMVictorSPX(PwmMotorIdConstants.INTAKE_BALL_MOTOR);
  private final Joystick m_stick;

  public Intake(Joystick stick) 
  {
    m_stick = stick;
  }

  @Override
  public void periodic() {
    if (m_stick.getRawButton(XboxConstants.B_BUTTON)) {
      setBelts(1);
    }
    else { 
      setBelts(0); 
    }
    
    if (m_stick.getRawButton(XboxConstants.A_BUTTON)) {
      setIntake(0.5);
    }
    else {
      setIntake(0);
    }

    intakeLiftMotor.set(m_stick.getRawAxis(XboxConstants.RIGHT_STICK_X_AXIS) * 0.25);
  }

  @Override
  public void simulationPeriodic() {}

  public void setBelts(double speed) {
    lowerBeltMotor.set(speed);
    upperBeltMotor.set(speed);
  }

  public void setIntake(double speed) {
    intakeBallMotor.set(speed);
  }
}
