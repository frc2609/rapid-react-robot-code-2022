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
  private final CANSparkMax rightHoodMotor = new CANSparkMax(Constants.CanMotorId.RIGHT_HOOD_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax leftHoodMotor = new CANSparkMax(Constants.CanMotorId.LEFT_HOOD_MOTOR,
      MotorType.kBrushless);

  private SparkMaxPIDController rightFlywheelPIDController;
  private SparkMaxPIDController hoodPIDController;
  private SparkMaxPIDController rotatePIDController;
  private boolean isAutoAimMode = false;
  private boolean pov_pressed = false;
  private double kD_LastError = 0.0;
  private long kD_LastTime = 0;
  private LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  public boolean isClimbingFullRotate = false;
  public boolean isClimbingLowRotate = false;
  public boolean isSpittingFullRotate = false;
  public boolean isSpittingLowRotate = false;
  public boolean isSpitting = false;

  private double manualFlywheelRpm = 0;
  private double autoFlywheelRpm = 0;
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
    rightHoodMotor.setInverted(true);
    rightFlywheelMotor.setInverted(false);
    leftFlywheelMotor.follow(rightFlywheelMotor, true);
    leftHoodMotor.follow(rightHoodMotor, true);

    rightFlywheelMotor.setIdleMode(IdleMode.kCoast);
    leftFlywheelMotor.setIdleMode(IdleMode.kCoast);
    rightHoodMotor.setIdleMode(IdleMode.kCoast);
    rotateMotor.setIdleMode(IdleMode.kBrake);

    rightFlywheelPIDController = rightFlywheelMotor.getPIDController();
    rotatePIDController = rotateMotor.getPIDController();
    hoodPIDController = rightHoodMotor.getPIDController();

    SmartDashboard.putNumber("Limelight camera angle (deg)", 32.2); //29.8
    SmartDashboard.putNumber("Shooter offset", 0);
    // SmartDashboard.putNumber("m", 80);
    // SmartDashboard.putNumber("b", 1800);

    // SmartDashboard.putNumber("kP Flywheel", Constants.Flywheel.PROPORTIONAL);
    // SmartDashboard.putNumber("kI Flywheel", Constants.Flywheel.INTEGRAL);
    // SmartDashboard.putNumber("kD Flywheel", Constants.Flywheel.DERIVATIVE);
    // SmartDashboard.putNumber("kIz Flywheel", Constants.Flywheel.INTEGRAL_ZONE);
    // SmartDashboard.putNumber("kFF Flywheel", Constants.Flywheel.FEED_FORWARD);

    // SmartDashboard.putNumber("kP Hood", Constants.Hood.PROPORTIONAL);
    // SmartDashboard.putNumber("kI Hood", Constants.Hood.INTEGRAL);
    // SmartDashboard.putNumber("kD Hood", Constants.Hood.DERIVATIVE);
    // SmartDashboard.putNumber("kIz Hood", Constants.Hood.INTEGRAL_ZONE);
    // SmartDashboard.putNumber("kFF Hood", Constants.Hood.FEED_FORWARD);

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

  public void setFlywheelAndHoodRpmPower(double power) {
    rightFlywheelMotor.set(power);
    rightHoodMotor.set(power);
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
    double offset = SmartDashboard.getNumber("Shooter offset", 0);
    double tx = txEntry.getDouble(0.0)+offset;

    return (
      isValidTarget
      && Math.abs(tx) < Constants.Rotate.TOLERANCE
      && Math.abs(autoFlywheelRpm * Constants.Flywheel.FLYWHEEL_OVERDRIVE - rightFlywheelMotor.getEncoder().getVelocity()) < Constants.AutoConstants.rpmTolerance
      && Math.abs(autoFlywheelRpm * Constants.Hood.HOOD_OVERDRIVE - rightHoodMotor.getEncoder().getVelocity()) < Constants.AutoConstants.rpmTolerance
    );
  }

  public void enableFlywheel() {
    isFlywheelDisabled = false;
  }
  public double getRPM(){
    return rightFlywheelMotor.getEncoder().getVelocity();
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

    hoodPIDController.setP(Constants.Hood.PROPORTIONAL);
    hoodPIDController.setI(Constants.Hood.INTEGRAL);
    hoodPIDController.setD(Constants.Hood.DERIVATIVE);
    hoodPIDController.setIZone(Constants.Hood.INTEGRAL_ZONE);
    hoodPIDController.setFF(Constants.Hood.FEED_FORWARD);
    hoodPIDController.setOutputRange(Constants.Hood.MIN_OUTPUT,
        Constants.Hood.MAX_OUTPUT);

    rotatePIDController.setP(Constants.Rotate.PROPORTIONAL);
    rotatePIDController.setI(Constants.Rotate.INTEGRAL);
    rotatePIDController.setD(Constants.Rotate.DERIVATIVE);
    rotatePIDController.setIZone(Constants.Rotate.INTEGRAL);
    rotatePIDController.setFF(Constants.Rotate.FEED_FORWARD);
    rotatePIDController.setOutputRange(Constants.Rotate.MIN_OUTPUT,
        Constants.Rotate.MAX_OUTPUT);
  }

  private void setPidValues_FlywheelTuning() {
    rightFlywheelPIDController.setP(SmartDashboard.getNumber("kP Flywheel", 0));
    rightFlywheelPIDController.setI(SmartDashboard.getNumber("kI Flywheel", 0));
    rightFlywheelPIDController.setD(SmartDashboard.getNumber("kD Flywheel", 0));
    rightFlywheelPIDController.setIZone(SmartDashboard.getNumber("kIz Flywheel", 0));
    rightFlywheelPIDController.setFF(SmartDashboard.getNumber("kFF Flywheel", 0));

    hoodPIDController.setP(SmartDashboard.getNumber("kP Hood", 0));
    hoodPIDController.setI(SmartDashboard.getNumber("kI Hood", 0));
    hoodPIDController.setD(SmartDashboard.getNumber("kD Hood", 0));
    hoodPIDController.setIZone(SmartDashboard.getNumber("kIz Hood", 0));
    hoodPIDController.setFF(SmartDashboard.getNumber("kFF Hood", 0));
  }

  public void turnLimelightOff() { table.getEntry("ledMode").setNumber(1); }

  public void turnLimelightOn() { table.getEntry("ledMode").setNumber(3); }

  private void autoFlywheel() {
    double distance = calcDistance();

    if (distance < 0) {
      return;
    }
    distance = Math.round(distance * 2) / 2;

    calcFlywheelRpm(distance);
    
    // changing the ratio for the hood:flywheel will change the rpm we need to set it to (i.e. the equation)
    rightFlywheelPIDController.setReference(autoFlywheelRpm * Constants.Flywheel.FLYWHEEL_OVERDRIVE, ControlType.kVelocity);
    hoodPIDController.setReference(autoFlywheelRpm * Constants.Hood.HOOD_OVERDRIVE, ControlType.kVelocity);

    autoRPMsetp = autoFlywheelRpm;
  }

  private void calcFlywheelRpm(double distance) {
    autoFlywheelRpm = 2.63*distance*distance + 62*distance + 1740 + autoFlywheelRpmTrim;
  }

  private double calcDistance() {
    double cameraHeight = 0.889; // height of camera in meters (from ground)
    double tapeHeight = 2.65; // height of retroreflective tape in meters (from ground)
    double tv = tvEntry.getDouble(0.0);
    double cameraAngleDegrees = SmartDashboard.getNumber("Limelight camera angle (deg)", 31.8);

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
          autoFlywheelRpmTrim -= 50;
          break;
        case Constants.Xbox.POV_RIGHT_BUTTON:
          pov_pressed = true;
          autoFlywheelRpmTrim += 50;
          break;
        default:
          break;
      }
    }

    if (pov == -1) {
      pov_pressed = false;
    }
  }

  public void rotateToPos(double pos){
    setPidValues();
    rotatePIDController.setReference(pos, ControlType.kSmartMotion);
  }

  public void shootWithBackspin(double rpm){
    rightFlywheelPIDController.setReference(rpm * Constants.Flywheel.FLYWHEEL_OVERDRIVE, ControlType.kVelocity);
    hoodPIDController.setReference(rpm * 0.25, ControlType.kVelocity);
  }

  public double getRotatePos(){
    return rotateMotor.getEncoder().getPosition();
  }

  private void autoRotate() {
    double kP = 0.02; // 0.017
    double frictionPower = 0.004; //0.004;  // 0.016 in + direction, -0.013 in - direction
    double rotatePower = 0.0;
    double isNegative = 1.0;
    boolean isValidTarget = tvEntry.getDouble(0.0) > 0.0;
    // double currTurretPosition = rotateMotor.getEncoder().getPosition();
    long currTime = System.nanoTime();
    double kD = 0.3;
    double offset = SmartDashboard.getNumber("Shooter offset", 0);

    if (!isValidTarget) {
      rotateMotor.set(0.0);
      return;
    }

    double tx = txEntry.getDouble(0.0)+offset;
    SmartDashboard.putNumber("tx", tx);

    if (Math.abs(tx) < Constants.Rotate.TOLERANCE) { 
      rotateMotor.set(0.0);
      return;
    }

    double rateError = (tx - kD_LastError) / (TimeUnit.NANOSECONDS.toMillis(currTime) - TimeUnit.NANOSECONDS.toMillis(kD_LastTime));
    rotatePower = tx*kP + frictionPower*isNegative + rateError*kD;
    rotateMotor.set(rotatePower);

    kD_LastError = tx;
    kD_LastTime = currTime;
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

    System.out.println("USING MANUAL FLYWHEEL");

    SmartDashboard.putNumber("Manual Flywheel RPM", manualFlywheelRpm);
    rightFlywheelPIDController.setReference(manualFlywheelRpm * Constants.Flywheel.FLYWHEEL_OVERDRIVE, ControlType.kVelocity);
    hoodPIDController.setReference(manualFlywheelRpm * Constants.Hood.HOOD_OVERDRIVE, ControlType.kVelocity);
  }

  private void manualSetRotatePower(Joystick stick) {
    double val = stick.getRawAxis(Constants.Xbox.RIGHT_STICK_X_AXIS);
    val = (Math.abs(val) < Constants.Xbox.JOYSTICK_DRIFT_TOLERANCE) ? 0 : val;

    rotateMotor.set(val / 4);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Actual Flywheel RPM", rightFlywheelMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Actual Hood RPM", rightHoodMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Auto Flywheel RPM Trim", autoFlywheelRpmTrim);
    SmartDashboard.putNumber("Auto Flywheel RPM", autoFlywheelRpm);
    SmartDashboard.putBoolean("Target Locked", isTargetLocked());
    SmartDashboard.putNumber("Left Flywheel Current", leftFlywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Right Flywheel Current", rightFlywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Right Hood Current", rightHoodMotor.getOutputCurrent());
    SmartDashboard.putNumber("Left Hood Current", leftHoodMotor.getOutputCurrent());
    SmartDashboard.putNumber("Turret pos", rotateMotor.getEncoder().getPosition());

    // SmartDashboard.putNumber("Actual Rotate Position", rotateMotor.getEncoder().getPosition());
    // SmartDashboard.putBoolean("Climbing", isClimbingFullRotate || isClimbingLowRotate);
    // SmartDashboard.putBoolean("Autoaim Enabled", isAutoAimMode);
    // //SmartDashboard.putNumber("Rotate Motor Current", rotateMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Intake Sensor Proximity", intakeSensor.getProximity());

    // setPidValues_FlywheelTuning();  // TODO: remove when done tuning flywheel and hood

    if (isClimbingFullRotate || isSpittingFullRotate) {
      rotateMotor.setSmartCurrentLimit(1); // prevent motor from burning itself out
      rotateMotor.set(-1.0);
      return;
    } else if (isClimbingLowRotate || isSpittingLowRotate) {
      rotateMotor.setSmartCurrentLimit(1);
      rotateMotor.set(-0.1);
      return;
    } else {
      rotateMotor.setSmartCurrentLimit(20);
    }

    // keeps autoAim enabled for rotate in auto mode so the robot doesn't lose track of the target
    if (isAutoAimMode && !isFlywheelDisabled) {
      autoAim();
    } else if(isAutoAimMode && isFlywheelDisabled && !isSpitting){
      rightFlywheelMotor.set(0);
      rightHoodMotor.set(0);
      autoRotate();
    } else if(!isAutoAimMode && !isSpitting) {
      manualAim();
    }
  }
}
