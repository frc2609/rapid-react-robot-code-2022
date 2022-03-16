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
  private RelativeEncoder rightFlywheelEncoder;
  private RelativeEncoder rotateEncoder;
  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController rightFlywheelPIDController;
  private SparkMaxPIDController rotatePIDController;
  private SparkMaxPIDController hoodPIDController;
  private boolean pov_pressed = false;
  private boolean isAutoAimMode = false;

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

  private void generateLookupTable() {
    // {key, value} = {distance (ft), [shooter RPM, hood pos]}
    lookupTable.put(0, new Double[] {0.0, 0.0});
    lookupTable.put(1, new Double[] {3276.923076923077, 0.0});
    lookupTable.put(2, new Double[] {3353.846153846154, 0.0});
    lookupTable.put(3, new Double[] {3430.769230769231, 0.0});
    lookupTable.put(4, new Double[] {3507.6923076923076, 0.0});
    lookupTable.put(5, new Double[] {3584.6153846153848, 0.0});
    lookupTable.put(6, new Double[] {3661.5384615384614, 0.0});
    lookupTable.put(7, new Double[] {3738.4615384615386, 0.10384615384615385});
    lookupTable.put(8, new Double[] {3815.3846153846152, 0.2076923076923077});
    lookupTable.put(9, new Double[] {3892.3076923076924, 0.31153846153846154});
    lookupTable.put(10, new Double[] {3969.230769230769, 0.4153846153846154});
    lookupTable.put(11, new Double[] {4046.153846153846, 0.5192307692307693});
    lookupTable.put(12, new Double[] {4323.076923076923, 0.7230769230769231});
    lookupTable.put(13, new Double[] {4400.0, 0.826923076923077});
    lookupTable.put(14, new Double[] {4476.923076923077, 0.9307692307692308});
    lookupTable.put(15, new Double[] {4553.846153846154, 1.0346153846153847});
    lookupTable.put(16, new Double[] {4830.7692307692305, 1.2384615384615385});
    lookupTable.put(17, new Double[] {4907.692307692308, 1.3423076923076924});
    lookupTable.put(18, new Double[] {4984.615384615385, 1.4461538461538461});
    lookupTable.put(19, new Double[] {5061.538461538461, 1.55});
    lookupTable.put(20, new Double[] {5138.461538461538, 1.653846153846154});
    lookupTable.put(21, new Double[] {5315.384615384615, 1.9576923076923078});
    lookupTable.put(22, new Double[] {5392.307692307692, 2.0615384615384618});
    lookupTable.put(23, new Double[] {5469.2307692307695, 2.1653846153846157});
    lookupTable.put(24, new Double[] {5546.153846153846, 2.269230769230769});
    lookupTable.put(25, new Double[] {5623.076923076923, 2.373076923076923});
    lookupTable.put(26, new Double[] {5700.0, 2.476923076923077});
  
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
    double cameraAngleDegrees = 24.25; // angle of camera in degrees (from horizontal plane)  TODO: Find this angle
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
    double kP = 0.011;
    double frictionPower = 0; //0.004;  // 0.016 in + direction, -0.013 in - direction
    double rotatePower = 0.0;
    double isNegative = 1.0;
    boolean isValidTarget = tvEntry.getDouble(0.0) > 0.0;
    double currTurretPosition = rotateEncoder.getPosition();

    SmartDashboard.putNumber("Auto Rotate Position (actual)", currTurretPosition);

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

    rotatePower = tx*kP + frictionPower*isNegative;
    rotateMotor.set(rotatePower);
    SmartDashboard.putNumber("Auto Rotate Power (setpoint)", rotatePower);
  }

  private void autoSetFlywheelAndHood(double distance) {
    if (distance < 0) {
      return;
    }

    int index = (int)Math.round(distance);
    SmartDashboard.putNumber("Auto lookupTable index", index);

    double flywheelRpm;
    double hoodPos;

    if (lookupTable.containsKey(index)) {
      flywheelRpm = lookupTable.get(index)[0];
      hoodPos = lookupTable.get(index)[1];
      SmartDashboard.putBoolean("Auto lookupTable index exists", true);
    } else {
      SmartDashboard.putBoolean("Auto lookupTable index exists", false);
      return;
    }


    rightFlywheelPIDController.setReference(flywheelRpm, ControlType.kVelocity);
    hoodPIDController.setReference(hoodPos, ControlType.kPosition);

    SmartDashboard.putNumber("Auto flywheel RPM (setpoint)", flywheelRpm);
    SmartDashboard.putNumber("Auto hoodPos (setpoint)", hoodPos);
    
    SmartDashboard.putNumber("Auto Hood Position (actual)", hoodEncoder.getPosition());
    SmartDashboard.putNumber("Auto Shooter Set (actual rpm)", rightFlywheelEncoder.getVelocity());
  }

  private void testGetFrictionPower() {
    if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_4)) {
      tempFrictionPower += 0.001;
    }

    if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_1)) {
      tempFrictionPower -= 0.001;
    }

    rotateMotor.set(tempFrictionPower);
    SmartDashboard.putNumber("TEST Friction power", tempFrictionPower);
  }

  @Override
  public void periodic() {
    // Run testGetFrictionPower() first and set frictionPower accordingly
    // testGetFrictionPower();

    if (m_stick.getRawButtonPressed(Constants.Logitech.BUTTON_4)) {
      if (isAutoAimMode) {
        isAutoAimMode = false;
        turnLimelightOff();
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
      autoSetFlywheelAndHood(calcDistance());
      autoRotateShooter_PowerControl();

    } else {
      manualSetFlywheelRpm();
      // manualSetHoodPos();
      manualSetRotatePower();
    }
  }
}
