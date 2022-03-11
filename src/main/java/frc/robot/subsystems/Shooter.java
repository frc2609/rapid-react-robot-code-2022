package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;

import javax.naming.ldap.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leftFlywheelMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_LEFT_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax rightFlywheelMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_RIGHT_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax rotateMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_ROTATE_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_HOOD_MOTOR,
      MotorType.kBrushless);
  private Joystick m_stick;
  private double flywheelRpm = 0; // double to avoid integer division
  private RelativeEncoder rightFlywheelEncoder;
  private RelativeEncoder rotateEncoder;
  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController rightFlywheelPIDController;
  private SparkMaxPIDController rotatePIDController;
  private SparkMaxPIDController hoodPIDController;
  private boolean m_pressed = false;

  double cameraHeight = 0.864; // height of camera in meters (from ground)
  double tapeHeight = 2.65; // height of retroreflective tape in meters (from ground)
  double cameraAngleDegrees = 60; // angle of camera in degrees
  double cameraAndTapeAngleDeltaDegrees; // difference in vertical angle between camera and target
  double distance;
  double ty;
  double tx;
  double tv;
  double shooterPosition;
  double hoodPos = 0;
  double rotatePos = 0;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("limelight");
  NetworkTableEntry txEntry = table.getEntry("tx");
  NetworkTableEntry tyEntry = table.getEntry("ty");
  NetworkTableEntry tvEntry = table.getEntry("tv");

  public Shooter(Joystick stick) {
    m_stick = stick;
    leftFlywheelMotor.follow(rightFlywheelMotor, true);

    rightFlywheelMotor.setIdleMode(IdleMode.kCoast);
    leftFlywheelMotor.setIdleMode(IdleMode.kCoast);

    rightFlywheelEncoder = rightFlywheelMotor.getEncoder();
    rotateEncoder = rotateMotor.getEncoder();
    hoodEncoder = hoodMotor.getEncoder();

    shooterPosition = 0.0;

    rightFlywheelPIDController = rightFlywheelMotor.getPIDController();
    rotatePIDController = rotateMotor.getPIDController();
    hoodPIDController = hoodMotor.getPIDController();

    rightFlywheelPIDController.setP(Constants.Flywheel.proportialPIDConstant);
    rightFlywheelPIDController.setI(Constants.Flywheel.integralPIDConstant);
    rightFlywheelPIDController.setD(Constants.Flywheel.derivativePIDConstant);
    rightFlywheelPIDController.setIZone(Constants.Flywheel.integralPIDZone);
    rightFlywheelPIDController.setFF(Constants.Flywheel.FeedForwardPIDConstant);
    rightFlywheelPIDController.setOutputRange(Constants.Flywheel.minPIDOutput,
        Constants.Flywheel.maxPIDOutput);

    rotatePIDController.setP(Constants.Rotate.proportialPIDConstant);
    rotatePIDController.setI(Constants.Rotate.integralPIDConstant);
    rotatePIDController.setD(Constants.Rotate.derivativePIDConstant);
    rotatePIDController.setIZone(Constants.Rotate.integralPIDConstant);
    rotatePIDController.setFF(Constants.Rotate.feedForwardPIDConstant);
    rotatePIDController.setOutputRange(Constants.Rotate.minPIDOutput,
        Constants.Rotate.maxPIDOutput);

    hoodPIDController.setP(Constants.Hood.proportialPIDConstant);
    hoodPIDController.setI(Constants.Hood.integralPIDConstant);
    hoodPIDController.setD(Constants.Hood.derivativePIDConstant);
    hoodPIDController.setIZone(Constants.Hood.integralPIDConstant);
    hoodPIDController.setFF(Constants.Hood.feedForwardPIDConstant);
    hoodPIDController.setOutputRange(Constants.Hood.minPIDOutput,
        Constants.Hood.maxPIDOutput);

    stopMotors();
    resetMotorEncoders();
  }

  public void stopMotors() {
    rightFlywheelMotor.set(0.0);
    rotateMotor.set(0.0);
    hoodMotor.set(0.0);
  }

  public void resetMotorEncoders() {
    hoodEncoder.setPosition(0.0);
    rotateEncoder.setPosition(0.0);
  }

  public void autoRotateShooter() {
    tx = txEntry.getDouble(0.0);
    shooterPosition = Math.min(Math.max(shooterPosition + tx, Constants.Rotate.MIN_TURRET_POS),
        Constants.Rotate.MAX_TURRET_POS) / 360;
    
    rotatePIDController.setReference(shooterPosition, ControlType.kPosition);
    SmartDashboard.putNumber("Rotate Pos (set)", shooterPosition);
    SmartDashboard.putNumber("Rotate Pos (actual)", rotateEncoder.getPosition());
  }

  private double degToRad(double degrees) {
    return degrees * Math.PI / 180;
  }

  private void calcDistance() {
    tv = tvEntry.getDouble(0.0);

    if (tv <= 0) {
      SmartDashboard.putBoolean("valid limelight target", false);
      return;
    }

    SmartDashboard.putBoolean("valid limelight target", true);

    tx = txEntry.getDouble(0);
    ty = tyEntry.getDouble(0);

    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);

    cameraAndTapeAngleDeltaDegrees = ty;
    distance = (tapeHeight - cameraHeight)
        / Math.tan(degToRad(cameraAngleDegrees) + degToRad(cameraAndTapeAngleDeltaDegrees));

    SmartDashboard.putNumber("limelight distance (m)", distance);
  }

  private void manualSetFlywheelRpm() {
    if (m_stick.getRawButtonPressed(Constants.Logitech.RIGHT_TOP_BUMPER)) {
      flywheelRpm += 100;
    }

    if (m_stick.getRawButtonPressed(Constants.Logitech.LEFT_TOP_BUMPER)) {
      flywheelRpm -= 100;
    }

    if (m_stick.getRawButtonPressed(Constants.Logitech.RIGHT_BOTTOM_BUMPER)) {
      flywheelRpm += 200;
    }

    if (m_stick.getRawButtonPressed(Constants.Logitech.LEFT_BOTTOM_BUMPER)) {
      flywheelRpm -= 200;
    }

    if (m_stick.getRawButtonPressed(Constants.Logitech.START_BUTTON)) {
      flywheelRpm = 0;
    }

    if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_4)) {
      flywheelRpm = 4600;
    }

    if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_1)) {
      flywheelRpm = 1600;
    }

    SmartDashboard.putNumber("Shooter Set (actual rpm)", rightFlywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Set (setpoint rpm)", flywheelRpm);

    rightFlywheelPIDController.setReference(flywheelRpm, ControlType.kVelocity);
  }

  private void manualSetHoodPos() {
    int pov = m_stick.getPOV();
    SmartDashboard.putNumber("Joystick POV", pov);

    if (!m_pressed) {
      switch (pov) {
        case Constants.Logitech.POV_UP_BUTTON:
          m_pressed = true;
          hoodPos += 0.1;
          break;
        case Constants.Logitech.POV_DOWN_BUTTON:
          m_pressed = true;
          hoodPos -= 0.1;
          break;
        default:
          break;
      }
    }

    if (pov == -1) {
      m_pressed = false;
    }

    hoodPos = Math.max(Math.min(hoodPos, Constants.Hood.MAX_POS), Constants.Hood.MIN_POS);

    hoodPIDController.setReference(hoodPos, ControlType.kPosition);

    SmartDashboard.putNumber("Hood Position (setpoint)", hoodPos);
    SmartDashboard.putNumber("Hood Position (actual)", hoodEncoder.getPosition());
  }

  private void manualSetRotateVelocity() {
    double val = m_stick.getRawAxis(Constants.Logitech.RIGHT_STICK_X_AXIS);

    val = (Math.abs(val) < Constants.Logitech.JOYSTICK_DRIFT_TOLERANCE) ? 0 : val;

    SmartDashboard.putNumber("Rotate Velocity (setpoint)", val);
    SmartDashboard.putNumber("Rotate Velocity (actual)", rotateEncoder.getVelocity());

    rotateMotor.set(val / 4);
  }

  public void setHood() {
    hoodPIDController.setReference(Math.abs(hoodPos), ControlType.kPosition);
  }

  private void manualSetRotatePosition() {
    double val = m_stick.getRawAxis(Constants.Logitech.RIGHT_STICK_X_AXIS);

    rotatePos += (Math.abs(val) < Constants.Logitech.JOYSTICK_DRIFT_TOLERANCE) ? 0 : val;

    rotatePIDController.setReference(rotatePos, ControlType.kPosition);

    SmartDashboard.putNumber("Rotate Position (setpoint)", rotatePos);
    SmartDashboard.putNumber("Rotate Position (actual)", rotateEncoder.getPosition());
  }

  private void autoRotateShooterV2() {
    double rotateSetpoint = 0.0;
    tv = tvEntry.getDouble(0.0);

    if (tv <= 0.0) {
      rotateMotor.set(rotateSetpoint);
      return;
    }

    tx = txEntry.getDouble(0.0);

    // if (tx < 0.0) {
    //   rotateSetpoint = Math.pow(-tx/30, 0.5) * tx/40;
    // } else {
    //   rotateSetpoint = Math.pow(tx/30, 0.5) * tx/40;
    // }

    rotateSetpoint = tx/75;

    rotateMotor.set(rotateSetpoint);

    SmartDashboard.putNumber("Rotate Velocity (setpoint)", rotateSetpoint);
    SmartDashboard.putNumber("Rotate Velocity (actual)", rotateEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    manualSetFlywheelRpm();
    manualSetHoodPos();
    // manualSetRotateVelocity();
    autoRotateShooterV2();
    // manualSetRotatePosition();
    calcDistance();
  }
}
