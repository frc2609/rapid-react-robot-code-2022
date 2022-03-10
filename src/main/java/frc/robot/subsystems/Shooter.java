package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
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
  private final CANSparkMax shooterHoodMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_HOOD_MOTOR,
      MotorType.kBrushless);
  private Joystick m_stick;
  private double flywheelRpm = 0; // double to avoid integer division
  private RelativeEncoder rightFlywheelEncoder;
  private RelativeEncoder rotateEncoder;
  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController rightFlywheelPIDController;
  private SparkMaxPIDController rotatePIDController;
  private SparkMaxPIDController hoodPIDController;

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
        Constants.Rotate.MAX_TURRET_POS);
    rotatePIDController.setReference((shooterPosition / 360), ControlType.kPosition);
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

    tx = tyEntry.getDouble(0);
    ty = tyEntry.getDouble(0);

    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);

    cameraAndTapeAngleDeltaDegrees = ty;
    distance = (tapeHeight - cameraHeight)
        / Math.tan(degToRad(cameraAngleDegrees) + degToRad(cameraAndTapeAngleDeltaDegrees));

    SmartDashboard.putNumber("limelight distance (m)", distance);
  }

  private void manualSetFlywheelRpm() {
    int pov = m_stick.getPOV();
    SmartDashboard.putNumber("Joystick POV", pov);

    // might be worth using the bottom bumper buttons on the logitech controller instead of the D-pad

    switch (pov) {
      case Constants.Logitech.POV_LEFT_BUTTON:
        flywheelRpm -= 100;
        break;
      case Constants.Logitech.POV_RIGHT_BUTTON:
        flywheelRpm += 100;
        break;
      default:
        break;
    }

    if(m_stick.getRawButtonPressed(Constants.Logitech.RIGHT_BUMPER)) {
      flywheelRpm += 200;
    }

    if(m_stick.getRawButtonPressed(Constants.Logitech.LEFT_BUMPER)) {
      flywheelRpm -= 200;
    }

    if(m_stick.getRawButtonPressed(Constants.Logitech.START_BUTTON)) {
      flywheelRpm = 0;
    }

    if(m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_4)) {
      flywheelRpm = 4600;
    }

    if(m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_1)) {
      flywheelRpm = 1600;
    }

    SmartDashboard.putNumber("Shooter Set (actual rpm)", rightFlywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Set (setpoint rpm)", flywheelRpm);

    rightFlywheelPIDController.setReference(flywheelRpm, ControlType.kVelocity);
  }

  private void manualSetHoodPos() {
    int pov = m_stick.getPOV();
    SmartDashboard.putNumber("Joystick POV", pov);

    switch (pov) {
      case Constants.Logitech.POV_UP_BUTTON:
        hoodPos += 0.1;
        break;
      case Constants.Logitech.POV_DOWN_BUTTON:
        hoodPos -= 0.1;
        break;
      default:
        break;
    }

    hoodPos = Math.max(Math.min(hoodPos, Constants.Hood.MAX_POS), Constants.Hood.MIN_POS);

    hoodPIDController.setReference(hoodPos, ControlType.kPosition);

    SmartDashboard.putNumber("Hood Position (setpoint)", hoodPos);
    SmartDashboard.putNumber("Hood Position (actual)", hoodEncoder.getPosition());
  }

  private void manualSetRotate() {
    double val = m_stick.getRawAxis(Constants.Logitech.RIGHT_STICK_X_AXIS);

    val = (val < Constants.Logitech.JOYSTICK_DRIFT_TOLERANCE) ? 0 : val;
    
    SmartDashboard.putNumber("Rotate Velocity (setpoint)", val);
    SmartDashboard.putNumber("Rotate Velocity (actual)", rotateEncoder.getVelocity());

    rotateMotor.set(val/4);
  }

  public void setHood() {
    hoodPIDController.setReference(Math.abs(hoodPos), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    manualSetFlywheelRpm();
    manualSetHoodPos();
    manualSetRotate();
    calcDistance();
  }
}
