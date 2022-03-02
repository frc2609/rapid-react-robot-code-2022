package frc.robot.subsystems;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPidConstants;
import frc.robot.Constants.XboxConstants;


public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterLeftMotor = new CANSparkMax(Constants.SHOOTER_LEFT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax shooterRightMotor = new CANSparkMax(Constants.SHOOTER_RIGHT_MOTOR, MotorType.kBrushless);
  private Joystick m_stick;
  private boolean m_pressed = false;
  private double m_speed = 0; // double to avoid integer division
  private RelativeEncoder rightMotorEncoder;
  private RelativeEncoder leftMotorEncoder;
  private SparkMaxPIDController rightPIDController;
  private SparkMaxPIDController leftPIDController;

  double h1 = 0.045; // height of camera in meters (from ground)
  double h2 = 0.23; // height of retroreflective tape in meters (from ground)
  double a1 = 0 * (Math.PI / 180.0); // angle of camera in degrees to radians
  double a2;
  double distance;
  double distanceH;
  double numerator;
  double denominator;
  double rpm;
  double metersPerSecond;
  double angle_from_example_calc = 56.78;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("limelight");
  NetworkTableEntry txEntry = table.getEntry("tx");
  NetworkTableEntry tyEntry = table.getEntry("ty");

  public Shooter(Joystick stick) {
    m_stick = stick;
    SmartDashboard.putNumber("Shooter Set", 0);
    shooterLeftMotor.follow(shooterRightMotor, true);
    leftMotorEncoder = shooterLeftMotor.getEncoder();
    rightMotorEncoder = shooterRightMotor.getEncoder();

    leftPIDController = shooterLeftMotor.getPIDController();
    rightPIDController = shooterRightMotor.getPIDController();

    leftPIDController.setP(ShooterPidConstants.proportialPIDConstant);
    leftPIDController.setI(ShooterPidConstants.integralPIDConstant);
    leftPIDController.setD(ShooterPidConstants.derivativePIDConstant);
    leftPIDController.setIZone(ShooterPidConstants.integralPIDConstant);
    leftPIDController.setFF(ShooterPidConstants.leftFeedForwardPIDConstant);
    leftPIDController.setOutputRange(ShooterPidConstants.minPIDOutput, ShooterPidConstants.maxPIDOutput);

    rightPIDController.setP(ShooterPidConstants.proportialPIDConstant);
    rightPIDController.setI(ShooterPidConstants.integralPIDConstant);
    rightPIDController.setD(ShooterPidConstants.derivativePIDConstant);
    rightPIDController.setIZone(ShooterPidConstants.integralPIDConstant);
    rightPIDController.setFF(ShooterPidConstants.rightFeedForwardPIDConstant);
    rightPIDController.setOutputRange(ShooterPidConstants.minPIDOutput, ShooterPidConstants.maxPIDOutput);
    stop();

    shooterLeftMotor.burnFlash();
    shooterRightMotor.burnFlash();
  }

  public void stop() {
    shooterRightMotor.set(0.0);
  }

  public void setVelocity() {
    double tx = txEntry.getDouble(0.0);
    double ty = tyEntry.getDouble(0.0);
    System.out.println("tx: " + tx + " ty: " + ty);

    a2 = ty * (Math.PI / 180.0);
    distance = (h2 - h1) / Math.tan(a1 + a2);
    System.out.println("distance: " + distance);

    distanceH = Math.sqrt(Math.pow(distance, 2) - Math.pow(1.878, 2));
    System.out.println("distanceH:" + distanceH);
    numerator = -4.9 * Math.pow(distanceH, 2);
    denominator = (1.878 - Math.tan(angle_from_example_calc * (Math.PI / 180.0)) * distanceH)
        * Math.pow(Math.cos(angle_from_example_calc * (Math.PI / 180.0)), 2);
    metersPerSecond = Math.sqrt(numerator / denominator);

    rpm = metersPerSecond / 0.00524;
    System.out.println("rpm: " + rpm);
    rightPIDController.setReference(rpm, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // double move = m_stick.getRawAxis(Constants.RIGHT_TRIGGER_AXIS);
    // setMotors(Math.abs(move) < Constants.JOYSTICK_DRIFT_TOLERANCE ? 0 : move);
    int pov = m_stick.getPOV();
    SmartDashboard.putNumber("Joystick POV", pov);
    if (!m_pressed) {
      switch (pov) {
        case XboxConstants.POV_UP_BUTTON:
          m_pressed = true;
          m_speed += 10;
          break;
        case XboxConstants.POV_DOWN_BUTTON:
          m_pressed = true;
          m_speed -= 10;
          break;
        case XboxConstants.POV_LEFT_BUTTON:
          m_pressed = true;
          m_speed -= 100;
          break;
        case XboxConstants.POV_RIGHT_BUTTON:
          m_pressed = true;
          m_speed += 100;
          break;
      }
    }
    if (pov == -1)
      m_pressed = false;
    SmartDashboard.putNumber("Shooter Set (actual rpm)", rightMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Set (setpoint rpm)", m_speed);
    setVelocity();
  }

  public void setMotors(double set) {
    SmartDashboard.putNumber("Shooter Speed", set);
    shooterLeftMotor.set(-set);
    shooterRightMotor.set(set);
  }

}
