package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Utils;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leftFlywheelMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_LEFT_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax rightFlywheelMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_RIGHT_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax rotateMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_ROTATE_MOTOR,
      MotorType.kBrushless);

  private SparkMaxPIDController rightFlywheelPIDController;
  private SparkMaxPIDController rotatePIDController;
  private boolean isAutoAimMode = false;
  private boolean pov_pressed = false;
  private double kD_LastError = 0.0;
  private long kD_LastTime = 0;
  private LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  public boolean isClimbingFullRotate = false;
  public boolean isClimbingLowRotate = false;

  private double manualFlywheelRpm = 0;

  private double autoFlywheelRpmTrim = 0;

  public final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
  public final DigitalInput stagingSensor = new DigitalInput(0);
  public final DigitalInput shooterSensor = new DigitalInput(1);

  private boolean isFlywheelDisabled = false;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("limelight");
  private NetworkTableEntry txEntry = table.getEntry("tx");
  private NetworkTableEntry tyEntry = table.getEntry("ty");
  private NetworkTableEntry tvEntry = table.getEntry("tv");
  
  public double autoRPMsetp = 0;

  public Shooter() {
    rotateMotor.setInverted(true);
    rightFlywheelMotor.setInverted(false);
    leftFlywheelMotor.follow(rightFlywheelMotor, true);

    rightFlywheelMotor.setIdleMode(IdleMode.kCoast);
    leftFlywheelMotor.setIdleMode(IdleMode.kCoast);
    rotateMotor.setIdleMode(IdleMode.kBrake);

    rightFlywheelPIDController = rightFlywheelMotor.getPIDController();
    rotatePIDController = rotateMotor.getPIDController();

    SmartDashboard.putNumber("Limelight camera angle (deg)", 29.8);

    turnLimelightOff();
    setPidValues();
    stopAllMotors();
    resetMotorEncoders();
  }

  public void autoAim() {
    if(DriverStation.isTeleop()){
      trimAdjust(RobotContainer.operatorJoystick);
    }
    autoRotate();
    autoFlywheel();
  }

  public void setRPMTrim(double trimAmt){
    this.autoFlywheelRpmTrim = trimAmt;
  }

  public void manualAim() {
    manualSetFlywheelRpm(RobotContainer.operatorJoystick);
    manualSetRotatePower(RobotContainer.operatorJoystick);
  }

  public void enableAutoAim() {
    isAutoAimMode = true;
    turnLimelightOn();
  }

  public boolean getIntakeSensor(){
    return intakeSensor.getProximity() > Constants.AutoConstants.proxThreshold;
  }

  public void disableAutoAim() {
    isAutoAimMode = false;
    turnLimelightOff();
  }

  public boolean isAutoAim() {
    return isAutoAimMode;
  }

  public boolean isSweetSpot() {
    double distance = calcDistance();
    return distance > Constants.SweetSpot.MIN && distance < Constants.SweetSpot.MAX;
  }

  public void stopAllMotors() {
    rightFlywheelMotor.set(0.0);
    rotateMotor.set(0.0);
  }

  public void resetMotorEncoders() {
    rotateMotor.getEncoder().setPosition(0.0);
  }

  public boolean isTargetLocked(){
    boolean isValidTarget = tvEntry.getDouble(0.0) > 0.0;
    double tx = txEntry.getDouble(0.0);
    double distance = calcDistance();

    return (isValidTarget
      && Math.abs(tx) < Constants.Rotate.TOLERANCE
      && Math.abs(calcFlywheelRpm(distance) - rightFlywheelMotor.getEncoder().getVelocity()) < Constants.AutoConstants.rpmTolerance
    );
  }

  public void enableFlywheel() {
    isFlywheelDisabled = false;
  }

  public void disableFlywheel() {
    isFlywheelDisabled = true;
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
  }

  private void turnLimelightOff() {
    table.getEntry("ledMode").setNumber(1); // force LEDs off
  }

  private void turnLimelightOn() {
    table.getEntry("ledMode").setNumber(3); // force LEDs on
  }

  private double calcDistance() {
    double cameraHeight = 0.889; // height of camera in meters (from ground)
    double tapeHeight = 2.65; // height of retroreflective tape in meters (from ground)
    double tv = tvEntry.getDouble(0.0);
    double cameraAngleDegrees = SmartDashboard.getNumber("Limelight camera angle (deg)", 29.8);

    if (tv <= 0) {
      SmartDashboard.putBoolean("Valid Limelight Target", false);
      return -1.0;
    }

    SmartDashboard.putBoolean("Valid Limelight Target", true);

    double ty = filter.calculate(tyEntry.getDouble(0.0));

    double distance = 3.281 * (tapeHeight - cameraHeight)  // 3.281 feet per meter
        / Math.tan(Utils.degToRad(cameraAngleDegrees) + Utils.degToRad(ty));

    SmartDashboard.putNumber("Limelight Distance (ft)", distance);

    return distance;
  }

  // automatic shooter control methods

  private void trimAdjust(Joystick stick) {
    int pov = stick.getPOV();

    if (!pov_pressed) {
      switch (pov) {
        case Constants.Xbox.POV_LEFT_BUTTON:
          pov_pressed = true;
          autoFlywheelRpmTrim -= 100;
          break;
        case Constants.Xbox.POV_RIGHT_BUTTON:
          pov_pressed = true;
          autoFlywheelRpmTrim += 100;
          break;
        default:
          break;
      }
    }

    if (pov == -1) {
      pov_pressed = false;
    }
  }

  private void autoRotate() {
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
    //SmartDashboard.putNumber("rateError", rateError);

    rotatePower = tx*kP + frictionPower*isNegative + rateError*kD;

    rotateMotor.set(rotatePower);
    SmartDashboard.putNumber("Auto Rotate Power", rotatePower);

    kD_LastError = tx;
    kD_LastTime = currTime;
  }

  private void autoFlywheel() {
    double distance = calcDistance();

    if (distance < 0) {
      return;
    }
    distance = Math.round(distance);

    double flywheelRpm = calcFlywheelRpm(distance);

    rightFlywheelPIDController.setReference(flywheelRpm, ControlType.kVelocity);

    autoRPMsetp = flywheelRpm;
    SmartDashboard.putNumber("Auto Flywheel RPM", flywheelRpm);
  }

  private double calcFlywheelRpm(double distance) {
    // return 1.2*distance*distance + 105*distance + 2800 + autoFlywheelRpmTrim;
    return 91*distance + 1480;
  }

  // manual shooter control methods

  private void manualSetFlywheelRpm(Joystick stick) {
    int pov = stick.getPOV();

    if (!pov_pressed) {
      switch (pov) {
        case Constants.Xbox.POV_RIGHT_BUTTON:
          pov_pressed = true;
          manualFlywheelRpm += 200;
          break;
        case Constants.Xbox.POV_LEFT_BUTTON:
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

    manualFlywheelRpm = Math.max(manualFlywheelRpm, 0);

    SmartDashboard.putNumber("Manual Flywheel RPM", manualFlywheelRpm);
    rightFlywheelPIDController.setReference(manualFlywheelRpm, ControlType.kVelocity);
  }

  private void manualSetRotatePower(Joystick stick) {
    double val = stick.getRawAxis(Constants.Xbox.RIGHT_STICK_X_AXIS);
    val = (Math.abs(val) < Constants.Xbox.JOYSTICK_DRIFT_TOLERANCE) ? 0 : val;

    SmartDashboard.putNumber("Manual Rotate Power", val);

    rotateMotor.set(val / 4);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Actual Flywheel RPM", rightFlywheelMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Actual Rotate Position", rotateMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Flywheel RPM Trim", autoFlywheelRpmTrim);
    SmartDashboard.putBoolean("Autoaim Enabled", isAutoAimMode);
    SmartDashboard.putBoolean("Target Locked", isTargetLocked());
    SmartDashboard.putBoolean("Climbing", isClimbingFullRotate || isClimbingLowRotate);
    SmartDashboard.putNumber("Rotate Motor Current", rotateMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake Sensor Proximity", intakeSensor.getProximity());

    if (isClimbingFullRotate) {
      rotateMotor.setSmartCurrentLimit(1); // prevent motor from burning itself out
      rotateMotor.set(-1.0);
      return;
    } else if (isClimbingLowRotate) {
      rotateMotor.setSmartCurrentLimit(1);
      rotateMotor.set(-0.1);
      return;
    } else {
      rotateMotor.setSmartCurrentLimit(20);
    }

    // keeps autoAim enabled for rotate in auto mode so the robot doesn't lose track of the target
    if (isAutoAimMode && !isFlywheelDisabled) {
      autoAim();
    } else if(isAutoAimMode && isFlywheelDisabled){
      rightFlywheelMotor.set(0);
      autoRotate();
    } else if(!isAutoAimMode) {
      manualAim();
    }
  }
}
