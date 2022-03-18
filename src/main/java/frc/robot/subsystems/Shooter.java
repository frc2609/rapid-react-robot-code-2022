package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Utils;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leftFlywheelMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_LEFT_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax rightFlywheelMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_RIGHT_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax rotateMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_ROTATE_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax hoodMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_HOOD_MOTOR,
      MotorType.kBrushless);

  private SparkMaxPIDController rightFlywheelPIDController;
  private SparkMaxPIDController rotatePIDController;
  private SparkMaxPIDController hoodPIDController;
  private boolean pov_pressed = false;
  private boolean isAutoAimMode = false;
  private double kD_LastError = 0.0;
  private long kD_LastTime = 0;

  private double manualHoodPos = 0;
  private double manualRotatePos = 0;
  private double manualFlywheelRpm = 0;

  private double autoHoodPosTrim = 0;
  private double autoFlywheelRpmTrim = 0;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("limelight");
  private NetworkTableEntry txEntry = table.getEntry("tx");
  private NetworkTableEntry tyEntry = table.getEntry("ty");
  private NetworkTableEntry tvEntry = table.getEntry("tv");

  public Shooter() {
    leftFlywheelMotor.follow(rightFlywheelMotor, true);

    rightFlywheelMotor.setIdleMode(IdleMode.kCoast);
    leftFlywheelMotor.setIdleMode(IdleMode.kCoast);

    rightFlywheelPIDController = rightFlywheelMotor.getPIDController();
    rotatePIDController = rotateMotor.getPIDController();
    hoodPIDController = hoodMotor.getPIDController();

    setPidValues();
    stopAllMotors();
    resetMotorEncoders();
  }

  private void setPidValues() {
    rightFlywheelPIDController.setP(Constants.Flywheel.PROPORTIONAL);
    rightFlywheelPIDController.setI(Constants.Flywheel.INTEGRAL);
    rightFlywheelPIDController.setD(Constants.Flywheel.DERIVATIVE);
    rightFlywheelPIDController.setIZone(Constants.Flywheel.INTEGRAL_ZONE);
    rightFlywheelPIDController.setFF(Constants.Flywheel.FEED_FORWARD);
    rightFlywheelPIDController.setOutputRange(Constants.Flywheel.MIN_OUTPUT,
        Constants.Flywheel.MAX_OUTPUT);

    rotatePIDController.setP(Constants.Rotate.PROPORTIONAL);
    rotatePIDController.setI(Constants.Rotate.INTEGRAL);
    rotatePIDController.setD(Constants.Rotate.DERIVATIVE);
    rotatePIDController.setIZone(Constants.Rotate.INTEGRAL);
    rotatePIDController.setFF(Constants.Rotate.FEED_FORWARD);
    rotatePIDController.setOutputRange(Constants.Rotate.MIN_OUTPUT,
        Constants.Rotate.MAX_OUTPUT);

    hoodPIDController.setP(Constants.Hood.PROPORTIONAL);
    hoodPIDController.setI(Constants.Hood.INTEGRAL);
    hoodPIDController.setD(Constants.Hood.DERIVATIVE);
    hoodPIDController.setIZone(Constants.Hood.INTEGRAL);
    hoodPIDController.setFF(Constants.Hood.FEED_FORWARD);
    hoodPIDController.setOutputRange(Constants.Hood.MIN_OUTPUT,
        Constants.Hood.MAX_OUTPUT);
  }

  public void stopAllMotors() {
    rightFlywheelMotor.set(0.0);
    rotateMotor.set(0.0);
    hoodMotor.set(0.0);
  }

  public void resetMotorEncoders() {
    hoodMotor.getEncoder().setPosition(0.0);
    rotateMotor.getEncoder().setPosition(0.0);
  }

  private void turnLimelightOff() {
    table.getEntry("ledMode").setNumber(1);
  }

  private void turnLimelightOn() {
    table.getEntry("ledMode").setNumber(3);
  }

  private double calcDistance() {
    double cameraHeight = 0.9017; // height of camera in meters (from ground)
    double tapeHeight = 2.65; // height of retroreflective tape in meters (from ground)
    double cameraAngleDegrees = 27.8; // angle of camera in degrees (from horizontal plane)
    double tv = tvEntry.getDouble(0.0);

    if (tv <= 0) {
      SmartDashboard.putBoolean("valid limelight target", false);
      return -1.0;
    }

    SmartDashboard.putBoolean("valid limelight target", true);

    double ty = tyEntry.getDouble(0.0);
    SmartDashboard.putNumber("ty", ty);

    double distance = 3.281 * (tapeHeight - cameraHeight)  // 3.281 feet per meter
        / Math.tan(Utils.degToRad(cameraAngleDegrees) + Utils.degToRad(ty));

    SmartDashboard.putNumber("limelight distance (ft)", distance);

    return distance;
  }

  private void manualSetFlywheelRpm(Joystick stick) {
    int pov = stick.getPOV();

    if (!pov_pressed) {
      switch (pov) {
        case Constants.Logitech.POV_RIGHT_BUTTON:
          pov_pressed = true;
          manualFlywheelRpm += 200;
          break;
        case Constants.Logitech.POV_LEFT_BUTTON:
          pov_pressed = true;
          manualFlywheelRpm -= 200;
          break;
        default:
          break;
      }
    }

    if (pov == -1) {
      pov_pressed = false;
    }

    SmartDashboard.putNumber("Manual Shooter Set (setpoint rpm)", manualFlywheelRpm);

    rightFlywheelPIDController.setReference(manualFlywheelRpm, ControlType.kVelocity);
  }

  private void manualSetHoodPos(Joystick stick) {
    int pov = stick.getPOV();

    if (!pov_pressed) {
      switch (pov) {
        case Constants.Logitech.POV_UP_BUTTON:
          pov_pressed = true;
          manualHoodPos += 0.1;
          break;
        case Constants.Logitech.POV_DOWN_BUTTON:
          pov_pressed = true;
          manualHoodPos -= 0.1;
          break;
        default:
          break;
      }
    }

    if (pov == -1) {
      pov_pressed = false;
    }

    manualHoodPos = Utils.clamp(manualHoodPos, Constants.Hood.MIN_POS, Constants.Hood.MAX_POS);
    hoodPIDController.setReference(manualHoodPos, ControlType.kPosition);
    
    SmartDashboard.putNumber("Manual Hood Position (setpoint)", manualHoodPos);
  }

  private void manualSetRotatePower(Joystick stick) {
    double val = stick.getRawAxis(Constants.Logitech.RIGHT_STICK_X_AXIS);
    val = (Math.abs(val) < Constants.Logitech.JOYSTICK_DRIFT_TOLERANCE) ? 0 : val;

    SmartDashboard.putNumber("Manual Rotate Power (setpoint)", val);

    rotateMotor.set(val / 4);
  }

  private void autoRotateShooter_PowerControl() {
    double kP = 0.02; // 0.017
    double frictionPower = 0.004; //0.004;  // 0.016 in + direction, -0.013 in - direction
    double rotatePower = 0.0;
    double isNegative = 1.0;
    boolean isValidTarget = tvEntry.getDouble(0.0) > 0.0;
    double currTurretPosition = rotateMotor.getEncoder().getPosition();
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

  private void autoSetFlywheelAndHood_Equation() {
    double distance = calcDistance();

    if (distance < 0) {
      return;
    }
    distance = Math.round(distance);

    double flywheelRpm = 1*distance*distance + 100*distance + 3000;
    double hoodPos = Utils.clamp(0.003*distance*distance + 0.03*distance - 0.2, Constants.Hood.MIN_POS, Constants.Hood.MAX_POS);

    rightFlywheelPIDController.setReference(flywheelRpm, ControlType.kVelocity);
    hoodPIDController.setReference(hoodPos, ControlType.kPosition);

    SmartDashboard.putNumber("Auto flywheel RPM (setpoint)", flywheelRpm);
    SmartDashboard.putNumber("Auto hoodPos (setpoint)", hoodPos);
  }

  public void autoAim() {
    autoRotateShooter_PowerControl();
    autoSetFlywheelAndHood_Equation();
  }

  public boolean isAutoAim() {
    return isAutoAimMode;
  }

  public void toggleAutoAimMode() {
    if (isAutoAimMode) {
      isAutoAimMode = false;
      turnLimelightOff();
      manualHoodPos = hoodMotor.getEncoder().getPosition();
      manualRotatePos = rotateMotor.getEncoder().getPosition();
    } else {
      isAutoAimMode = true;
      turnLimelightOn();
    }
  }

  public void manualAim(Joystick stick) {
    manualSetFlywheelRpm(stick);
    manualSetHoodPos(stick);
    manualSetRotatePower(stick);
  }

  @Override
  public void periodic() {
  //   if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_4)) {
  //     if (isAutoAimMode) {
  //       isAutoAimMode = false;
  //       turnLimelightOff();
  //       hoodPos = hoodMotor.getEncoder().getPosition();
  //       rotatePos = rotateMotor.getEncoder().getPosition();
  //     } else {
  //       isAutoAimMode = true;
  //       turnLimelightOn();
  //     }
  //   }

  //   SmartDashboard.putBoolean("isAutoAimMode", isAutoAimMode);
    
  //   if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_1)) {
  //     resetMotorEncoders();
  //   }

  //   if (isAutoAimMode) {
  //     autoSetFlywheelAndHood_Equation();
  //     autoRotateShooter_PowerControl();

  //   } else {
  //     manualSetFlywheelRpm();
  //     manualSetHoodPos();
  //     manualSetRotatePower();
  //   }

  //   SmartDashboard.putNumber("Hood Position (actual)", hoodMotor.getEncoder().getPosition());
  //   SmartDashboard.putNumber("Flywheel RPM (actual)", rightFlywheelMotor.getEncoder().getVelocity());
  //   SmartDashboard.putNumber("Rotate Position (actual)", rotateMotor.getEncoder().getPosition());
  }
}
