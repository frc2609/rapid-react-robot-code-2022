package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final PWMVictorSPX lowerBeltMotor = new PWMVictorSPX(Constants.PwmMotorId.LOWER_BELT_MOTOR);
  private final PWMVictorSPX upperBeltMotor = new PWMVictorSPX(Constants.PwmMotorId.UPPER_BELT_MOTOR);
  private final PWMVictorSPX intakeLiftMotor = new PWMVictorSPX(Constants.PwmMotorId.INTAKE_LIFT_MOTOR);
  private final PWMVictorSPX intakeBallMotor = new PWMVictorSPX(Constants.PwmMotorId.INTAKE_BALL_MOTOR);
  private final Joystick m_stick;

  public Intake(Joystick stick) {
    m_stick = stick;
  }

  @Override
  public void periodic() {
    /*
     * if (m_stick.getRawButton(Constants.Logitech.BUTTON_3)) {
     * setBelts(1);
     * }
     * else {
     * //setBelts(0);
     * upperBeltMotor.set(0);
     * }
     * 
     * if (m_stick.getRawButton(Constants.Logitech.BUTTON_2)) {
     * setIntake(0.5);
     * }
     * else {
     * setIntake(0);
     * }
     * 
     * intakeLiftMotor.set(m_stick.getRawAxis(Constants.Logitech.RIGHT_STICK_X_AXIS)
     * * 0.25);
     */}

  @Override
  public void simulationPeriodic() {
  }

  public void setBelts(double speed) {
    lowerBeltMotor.set(speed);
    upperBeltMotor.set(speed);
  }

  public void setIntake(double speed) {
    intakeBallMotor.set(speed);
  }
}
