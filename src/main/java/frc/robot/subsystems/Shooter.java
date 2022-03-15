package frc.robot.subsystems;

import java.util.HashMap;

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
  private Joystick m_stick;
  private double flywheelRpm = 0; // double to avoid integer division
  private RelativeEncoder rightFlywheelEncoder;
  private RelativeEncoder rotateEncoder;
  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController rightFlywheelPIDController;
  private SparkMaxPIDController rotatePIDController;
  private SparkMaxPIDController hoodPIDController;
  private boolean m_pressed = false;
  private boolean isAutoAimMode = false;

  double shooterPosition;
  double hoodPos = 0;
  double rotatePos = 0;
  double tempFrictionPower = 0.0;

  HashMap<Integer, Double[]> lookupTable = new HashMap<Integer, Double[]>();

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

//#region Setting PID values
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
//#endregion
    
    generateLookupTable();
    stopAllMotors();
    resetMotorEncoders();
  }

  private void generateLookupTable() {
    // {key, value} = {distance (ft), [shooter RPM, hood pos]}
    lookupTable.put(1, new Double[] {0.0, 0.0});
    lookupTable.put(2, new Double[] {0.0, 0.0});
    lookupTable.put(3, new Double[] {0.0, 0.0});
    lookupTable.put(4, new Double[] {0.0, 0.0});
    lookupTable.put(5, new Double[] {0.0, 0.0});
    lookupTable.put(6, new Double[] {0.0, 0.0});
    lookupTable.put(7, new Double[] {0.0, 0.0});
    lookupTable.put(8, new Double[] {0.0, 0.0});
    lookupTable.put(9, new Double[] {0.0, 0.0});
    lookupTable.put(10, new Double[] {0.0, 0.0});
    lookupTable.put(11, new Double[] {0.0, 0.0});
    lookupTable.put(12, new Double[] {0.0, 0.0});
    lookupTable.put(13, new Double[] {0.0, 0.0});
    lookupTable.put(14, new Double[] {0.0, 0.0});
    lookupTable.put(15, new Double[] {0.0, 0.0});
    lookupTable.put(16, new Double[] {0.0, 0.0});
    lookupTable.put(17, new Double[] {0.0, 0.0});
    lookupTable.put(18, new Double[] {0.0, 0.0});
  }

  public void stopAllMotors() {
    rightFlywheelMotor.set(0.0);
    rotateMotor.set(0.0);
    hoodMotor.set(0.0);
  }

  public void resetMotorEncoders() {
    hoodEncoder.setPosition(0.0);
    rotateEncoder.setPosition(0.0);
  }

  private double degToRad(double degrees) {
    return degrees * Math.PI / 180;
  }

  private void turnLimelightOff() {
    table.getEntry("ledMode").setNumber(1);
  }

  private void turnLimelightOn() {
    table.getEntry("ledMode").setNumber(3);
  }

  private double calcDistance() {
    double cameraHeight = 0.864; // height of camera in meters (from ground)  TODO: Find this height
    double tapeHeight = 2.65; // height of retroreflective tape in meters (from ground)
    double cameraAngleDegrees = 60; // angle of camera in degrees (from horizontal plane)  TODO: Find this angle
    double tv = tvEntry.getDouble(0.0);

    if (tv <= 0) {
      SmartDashboard.putBoolean("valid limelight target", false);
      return -1.0;
    }

    SmartDashboard.putBoolean("valid limelight target", true);

    double ty = tyEntry.getDouble(0.0);
    SmartDashboard.putNumber("ty", ty);

    double distance = 3.281 * (tapeHeight - cameraHeight)  // 3.281 feet per meter
        / Math.tan(degToRad(cameraAngleDegrees) + degToRad(ty));

    SmartDashboard.putNumber("limelight distance (ft)", distance);

    return distance;
  }
//#region Manual Controls
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

    // if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_4)) {
    //   flywheelRpm = 4600;
    // }

    // if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_1)) {
    //   flywheelRpm = 1600;
    // }

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

  private void manualSetRotatePower() {
    double val = m_stick.getRawAxis(Constants.Logitech.RIGHT_STICK_X_AXIS);

    val = (Math.abs(val) < Constants.Logitech.JOYSTICK_DRIFT_TOLERANCE) ? 0 : val;

    SmartDashboard.putNumber("Rotate Velocity (setpoint)", val);
    SmartDashboard.putNumber("Rotate Velocity (actual)", rotateEncoder.getVelocity());

    rotateMotor.set(val / 4);
  }

  private void manualSetRotatePosition() {  // deprecated for now
    double val = m_stick.getRawAxis(Constants.Logitech.RIGHT_STICK_X_AXIS);

    rotatePos += (Math.abs(val) < Constants.Logitech.JOYSTICK_DRIFT_TOLERANCE) ? 0 : val;

    rotatePIDController.setReference(rotatePos, ControlType.kPosition);

    SmartDashboard.putNumber("Rotate Position (setpoint)", rotatePos);
    SmartDashboard.putNumber("Rotate Position (actual)", rotateEncoder.getPosition());
  }
//#endregion
  
  public void autoRotateShooter_PositionControl() {
    // This code is untested
    double tx = txEntry.getDouble(0.0);
    shooterPosition = Math.min(Math.max(shooterPosition + tx, Constants.Rotate.MIN_POS),
        Constants.Rotate.MAX_POS) / 360;
    
    rotatePIDController.setReference(shooterPosition, ControlType.kPosition);
    SmartDashboard.putNumber("Rotate Pos (set)", shooterPosition);
    SmartDashboard.putNumber("Rotate Pos (actual)", rotateEncoder.getPosition());
  }

  private void autoRotateShooter_PowerControl() {
    double kP = 0.01333;
    double frictionPower = 0.0;  // TODO: Need to figure out what this would be
    double rotatePower = 0.0;
    double isNegative = 1.0;
    boolean isValidTarget = tvEntry.getDouble(0.0) > 0.0;
    double currTurretPosition = rotateEncoder.getPosition();
    SmartDashboard.putNumber("Rotate Position (actual)", currTurretPosition);

    if (!isValidTarget) {
      rotateMotor.set(rotatePower);
      return;
    }

    double tx = txEntry.getDouble(0.0);

    if (Math.abs(tx) < Constants.Rotate.TOLERANCE) { return; }

    if (tx < 0.0) {
      isNegative = -1.0;
      if (currTurretPosition <= Constants.Rotate.MIN_POS) { return; }
    }

    if (tx > 0.0) {
      isNegative = 1.0;
      if (currTurretPosition >= Constants.Rotate.MAX_POS) { return; }
    }

    rotatePower = tx*kP + frictionPower*isNegative;

    if (isAutoAimMode) {
      rotateMotor.set(rotatePower);
    }
    
    SmartDashboard.putNumber("Rotate Power (setpoint)", rotatePower);
  }

  private void testGetFrictionPower() {
    if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_4)) {
      tempFrictionPower += 0.01;
    }

    if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_1)) {
      tempFrictionPower -= 0.01;
    }

    SmartDashboard.putNumber("TEST Friction power", tempFrictionPower);
  }

  private void autoSetFlywheelAndHood(double distance) {
    if (distance < 0) {
      return;
    }

    int index = (int)Math.ceil(distance);
    double flywheelRpm = lookupTable.get(index)[0];
    double hoodPos = lookupTable.get(index)[1];

    if (isAutoAimMode) {
      rightFlywheelPIDController.setReference(flywheelRpm, ControlType.kVelocity);
      hoodPIDController.setReference(hoodPos, ControlType.kPosition);
    }

    SmartDashboard.putNumber("Auto flywheel RPM", flywheelRpm);
    SmartDashboard.putNumber("Auto hoodPos", hoodPos);
    SmartDashboard.putNumber("Auto Hood Position (actual)", hoodEncoder.getPosition());
    SmartDashboard.putNumber("Auto Shooter Set (actual rpm)", rightFlywheelEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    // Run testGetFrictionPower() first and set frictionPower accordingly
    testGetFrictionPower();

    if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_4)) {
      if (isAutoAimMode) {
        isAutoAimMode = false;
        turnLimelightOff();
      } else {
        isAutoAimMode = true;
        turnLimelightOn();
      }
    }

    if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_1)) {
      rotateEncoder.setPosition(0.0);
    }

    autoSetFlywheelAndHood(calcDistance());
    autoRotateShooter_PowerControl();

    if (!isAutoAimMode) {
      manualSetFlywheelRpm();
      manualSetHoodPos();
      manualSetRotatePower();
    }
  }
}
