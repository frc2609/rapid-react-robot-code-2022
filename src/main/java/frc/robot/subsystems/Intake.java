package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final PWMVictorSPX lowerBeltMotor = new PWMVictorSPX(Constants.PwmMotorId.LOWER_BELT_MOTOR);
  private final PWMVictorSPX upperBeltMotor = new PWMVictorSPX(Constants.PwmMotorId.UPPER_BELT_MOTOR);
  private final PWMVictorSPX intakeLiftMotor = new PWMVictorSPX(Constants.PwmMotorId.INTAKE_LIFT_MOTOR);
  private final PWMVictorSPX intakeBallMotor = new PWMVictorSPX(Constants.PwmMotorId.INTAKE_BALL_MOTOR);

  public Intake() {}

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
  }

  public void setBelts(double speed) {
    setLowerBelt(speed);
    setUpperBelt(speed);
  }

  public void setLowerBelt(double speed) {
    lowerBeltMotor.set(speed);
  }

  public void setUpperBelt(double speed) {
    upperBeltMotor.set(speed);
  }

  public void setIntake(double speed) {
    // intakeBallMotor.set(speed);
  }

  public void setIntakeLift(double speed) {
    intakeLiftMotor.set(speed * Constants.Motors.INTAKE_LIFT_SPEED);
  }
}
