package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorID.PWM;

public class Intake extends SubsystemBase {
  private final PWMVictorSPX lowerBeltMotor = new PWMVictorSPX(PWM.LOWER_BELT);
  private final PWMVictorSPX upperBeltMotor = new PWMVictorSPX(PWM.UPPER_BELT);
  private final PWMVictorSPX intakeLiftMotor = new PWMVictorSPX(PWM.INTAKE_LIFT);
  private final PWMVictorSPX intakeBallMotor = new PWMVictorSPX(PWM.INTAKE_BALL);
  private final double beltSpeed, intakeSpeed, intakeLiftSpeed;

  public Intake(double intakeSpeed, double intakeLiftSpeed, double beltSpeed) {
    this.intakeSpeed = MathUtil.clamp(intakeSpeed, 0, 1);
    this.intakeLiftSpeed = MathUtil.clamp(intakeLiftSpeed, 0, 1);
    this.beltSpeed = MathUtil.clamp(beltSpeed, 0, 1);
  }

  @Override
  public void periodic() {}

  public void allBeltsForward() {
    lowerBeltForward();
    upperBeltForward();
  }

  public void allBeltsReverse() {
    lowerBeltReverse();
    upperBeltReverse();
  }

  public void extendIntake() {
    intakeLiftMotor.set(intakeLiftSpeed); // check whether motor should be inverted or not
  }

  public void retractIntake() {
    intakeLiftMotor.set(-intakeLiftSpeed);
  }

  public void lowerBeltForward() {
    lowerBeltMotor.set(beltSpeed);
  }

  public void lowerBeltReverse() {
    lowerBeltMotor.set(-beltSpeed);
  }

  public void upperBeltForward() {
    upperBeltMotor.set(beltSpeed);
  }

  public void upperBeltReverse() {
    upperBeltMotor.set(-beltSpeed);
  }

  public void intake() {
    intakeBallMotor.set(intakeSpeed);
  }

  public void outtake() {
    intakeBallMotor.set(-intakeSpeed);
  }

  public void setIntakeLift(double set) {
    intakeLiftMotor.set(set);
  }

  public void stopBallMotor() {
    intakeBallMotor.stopMotor();
  }

  public void stopLiftMotor() {
    intakeLiftMotor.stopMotor();
  }

  public void stopLowerBelt() {
    lowerBeltMotor.stopMotor();
  }

  public void stopUpperBelt() {
    upperBeltMotor.stopMotor();
  }

  public void stopMotors() {
    intakeBallMotor.stopMotor();
    intakeLiftMotor.stopMotor();
    lowerBeltMotor.stopMotor();
    upperBeltMotor.stopMotor();
  }
}
