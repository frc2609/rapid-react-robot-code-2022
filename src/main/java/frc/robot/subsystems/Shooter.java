package frc.robot.subsystems;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.ejml.dense.block.linsol.chol.CholeskyOuterSolver_MT_DDRB;

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
  private RelativeEncoder rightFlywheelEncoder;
  private RelativeEncoder rotateEncoder;
  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController rightFlywheelPIDController;
  private SparkMaxPIDController rotatePIDController;
  private SparkMaxPIDController hoodPIDController;
  private boolean pov_pressed = false;
  private boolean isAutoAimMode = false;
  private double kD_LastError = 0.0;
  private long kD_LastTime = 0;

  // temporary variables for testing
  double shooterPosition;
  double hoodPos = 0;
  double rotatePos = 0;
  double tempFrictionPower = 0.0;
  double flywheelRpm = 0; // double to avoid integer division

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
    double cameraHeight = 0.9017; // height of camera in meters (from ground)  TODO: Find this height
    double tapeHeight = 2.65; // height of retroreflective tape in meters (from ground)
    double cameraAngleDegrees = 27.8; // angle of camera in degrees (from horizontal plane)  TODO: Find this angle
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

    SmartDashboard.putNumber("Shooter Set (setpoint rpm)", flywheelRpm);

    rightFlywheelPIDController.setReference(flywheelRpm, ControlType.kVelocity);
  }

  private void manualSetHoodPos() {
    int pov = m_stick.getPOV();
    SmartDashboard.putNumber("Joystick POV", pov);

    if (!pov_pressed) {
      switch (pov) {
        case Constants.Logitech.POV_UP_BUTTON:
          pov_pressed = true;
          hoodPos += 0.1;
          break;
        case Constants.Logitech.POV_DOWN_BUTTON:
          pov_pressed = true;
          hoodPos -= 0.1;
          break;
        default:
          break;
      }
    }

    if (pov == -1) {
      pov_pressed = false;
    }

    hoodPos = Math.max(Math.min(hoodPos, Constants.Hood.MAX_POS), Constants.Hood.MIN_POS);

    hoodPIDController.setReference(hoodPos, ControlType.kPosition);

    SmartDashboard.putNumber("Hood Position (setpoint)", hoodPos);
  }

  private void manualSetRotatePower() {
    double val = m_stick.getRawAxis(Constants.Logitech.RIGHT_STICK_X_AXIS);

    val = (Math.abs(val) < Constants.Logitech.JOYSTICK_DRIFT_TOLERANCE) ? 0 : val;

    SmartDashboard.putNumber("Rotate Power (setpoint)", val);

    rotateMotor.set(val / 4);
  }

//#endregion

  private void autoRotateShooter_PowerControl() {
    double kP = 0.02; // 0.017
    double frictionPower = 0.004; //0.004;  // 0.016 in + direction, -0.013 in - direction
    double rotatePower = 0.0;
    double isNegative = 1.0;
    boolean isValidTarget = tvEntry.getDouble(0.0) > 0.0;
    double currTurretPosition = rotateEncoder.getPosition();
    long currTime = System.nanoTime();
    double kD = 0.3;

    if (!isValidTarget) {
      rotateMotor.set(0.0);
      return;
    }

    double tx = txEntry.getDouble(0.0);
    SmartDashboard.putNumber("tx", tx);


    if (Math.abs(tx) < Constants.Rotate.TOLERANCE) { 
      rotateMotor.set(0.0);
      return;
    }

    if (tx < 0.0) {
      isNegative = -1.0;
      if (currTurretPosition <= Constants.Rotate.MIN_POS) {
        rotateMotor.set(0.0);
        return;
      }
    }

    if (tx > 0.0) {
      isNegative = 1.0;
      if (currTurretPosition >= Constants.Rotate.MAX_POS) {
        rotateMotor.set(0.0);
        return;
      }
    }

    double rateError = (tx - kD_LastError) / (TimeUnit.NANOSECONDS.toMillis(currTime) - TimeUnit.NANOSECONDS.toMillis(kD_LastTime));
    SmartDashboard.putNumber("rateError", rateError);

    rotatePower = tx*kP + frictionPower*isNegative + rateError*kD;

    rotateMotor.set(rotatePower);
    SmartDashboard.putNumber("Auto Rotate Power (setpoint)", rotatePower);

    kD_LastError = tx;
    kD_LastTime = currTime;
  }

  private void autoSetFlywheelAndHood_Equation(double distance) {
    if (distance < 0) {
      return;
    }
    distance = Math.round(distance);

    double flywheelRpm = 1*distance*distance + 100*distance + 3000;
    double hoodPos = Math.min(Math.max(0.003*distance*distance + 0.03*distance - 0.2, Constants.Hood.MIN_POS), Constants.Hood.MAX_POS);

    rightFlywheelPIDController.setReference(flywheelRpm, ControlType.kVelocity);
    hoodPIDController.setReference(hoodPos, ControlType.kPosition);

    SmartDashboard.putNumber("Auto flywheel RPM (setpoint)", flywheelRpm);
    SmartDashboard.putNumber("Auto hoodPos (setpoint)", hoodPos);
  }

  @Override
  public void periodic() {
    if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_4)) {
      if (isAutoAimMode) {
        isAutoAimMode = false;
        turnLimelightOff();
        hoodPos = hoodEncoder.getPosition();
        rotatePos = rotateEncoder.getPosition();
      } else {
        isAutoAimMode = true;
        turnLimelightOn();
      }
    }

    SmartDashboard.putBoolean("isAutoAimMode", isAutoAimMode);
    
    if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_1)) {
      resetMotorEncoders();
    }

    if (isAutoAimMode) {
      // autoSetFlywheelAndHood_LookupTable(calcDistance());
      autoSetFlywheelAndHood_Equation(calcDistance());
      autoRotateShooter_PowerControl();

    } else {
      manualSetFlywheelRpm();
      manualSetHoodPos();
      manualSetRotatePower();
    }

    SmartDashboard.putNumber("Hood Position (actual)", hoodEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Set (actual rpm)", rightFlywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Rotate Position (actual)", rotateEncoder.getPosition());
  }
}
