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

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterLeftMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_LEFT_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax shooterRightMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_RIGHT_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax shooterRotateMotor = new CANSparkMax(Constants.CanMotorId.SHOOTER_ROTATE_MOTOR,
      MotorType.kBrushless);
  private Joystick m_stick;
  private boolean m_pressed = false;
  private double m_speed = 0; // double to avoid integer division
  private RelativeEncoder rightMotorEncoder;
  private RelativeEncoder rotateMotorEncoder;
  private SparkMaxPIDController rightPIDController;
  private SparkMaxPIDController rotatePIDController;

  double cameraHeight = 0.92; // height of camera in meters (from ground)
  double tapeHeight = 2.65; // height of retroreflective tape in meters (from ground)
  double cameraAngleDegrees = 35; // angle of camera in degrees
  double cameraAndTapeAngleDeltaDegrees; // difference in vertical angle between camera and target
  double distance;
  double distanceH;
  double numerator;
  double denominator;
  double rpm;
  double metersPerSecond;
  double angle_from_example_calc = 0.0; //56.78;
  double ty;
  double tx;
  double tv;
  double shooterPosition;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("limelight");
  NetworkTableEntry txEntry = table.getEntry("tx");
  NetworkTableEntry tyEntry = table.getEntry("ty");
  NetworkTableEntry tvEntry = table.getEntry("tv");

  public Shooter(Joystick stick) {
    m_stick = stick;
    SmartDashboard.putNumber("Shooter Set", 0);
    shooterLeftMotor.follow(shooterRightMotor, true);
    rightMotorEncoder = shooterRightMotor.getEncoder();
    rotateMotorEncoder = shooterRotateMotor.getEncoder();

    rightPIDController = shooterRightMotor.getPIDController();
    rotatePIDController = shooterRotateMotor.getPIDController();

    rightPIDController.setP(Constants.ShooterPid.proportialPIDConstant);
    rightPIDController.setI(Constants.ShooterPid.integralPIDConstant);
    rightPIDController.setD(Constants.ShooterPid.derivativePIDConstant);
    rightPIDController.setIZone(Constants.ShooterPid.integralPIDConstant);
    rightPIDController.setFF(Constants.ShooterPid.rightFeedForwardPIDConstant);
    rightPIDController.setOutputRange(Constants.ShooterPid.minShooterPIDOutput,
        Constants.ShooterPid.maxShooterPIDOutput);

    rotatePIDController.setP(Constants.ShooterPid.proportialPIDConstant);
    rotatePIDController.setI(Constants.ShooterPid.integralPIDConstant);
    rotatePIDController.setD(Constants.ShooterPid.derivativePIDConstant);
    rotatePIDController.setIZone(Constants.ShooterPid.integralPIDConstant);
    rotatePIDController.setFF(Constants.ShooterPid.leftFeedForwardPIDConstant);
    rotatePIDController.setOutputRange(Constants.ShooterPid.minRotatePIDOutput,
        Constants.ShooterPid.maxRotatePIDOutput);

    stopShooterMotors();

    // shooterLeftMotor.burnFlash();
    // shooterRightMotor.burnFlash();
    // shooterRotateMotor.burnFlash();
  }

  public void stopShooterMotors() {
    shooterRightMotor.set(0.0);
    shooterRotateMotor.set(0.0);
  }

  // public void setVelocity() {
  //   tv = tvEntry.getDouble(0.0);

  //   if(tv <= 0) {
  //     SmartDashboard.putBoolean("valid limelight target", false);
  //     // rightPIDController.setReference(0, ControlType.kVelocity);
  //     return;
  //   }

  //   SmartDashboard.putBoolean("valid limelight target", true);
    
  //   tx = tyEntry.getDouble(0);
  //   ty = tyEntry.getDouble(0);

  //   SmartDashboard.putNumber("tx", tx);
  //   SmartDashboard.putNumber("ty", ty);


  //   cameraAndTapeAngleDeltaDegrees = ty * (Math.PI / 180.0);
  //   distance = (tapeHeight - cameraHeight) / Math.tan(degToRad(cameraAngleDegrees) + degToRad(cameraAndTapeAngleDeltaDegrees));

  //   System.out.println("distance: " + distance);
  //   SmartDashboard.putNumber("distance (m)", distance);


  //   distanceH = Math.sqrt(Math.pow(distance, 2) - Math.pow(1.878, 2));
  //   System.out.println("distanceH:" + distanceH);
  //   SmartDashboard.putNumber("distanceH (m)", distanceH);

  //   numerator = -4.9 * Math.pow(distanceH, 2);
  //   denominator = (1.878 - Math.tan(angle_from_example_calc * (Math.PI / 180.0)) * distanceH)
  //       * Math.pow(Math.cos(angle_from_example_calc * (Math.PI / 180.0)), 2);
  //   metersPerSecond = Math.sqrt(numerator / denominator);

  //   rpm = metersPerSecond / 0.00524;
  //   System.out.println("rpm: " + rpm);
  //   SmartDashboard.putNumber("flywheel rpm", rpm);
  //   rightPIDController.setReference(rpm, ControlType.kVelocity);
  // }

  private void fernCalculateFlywheelRpm() {
    tv = tvEntry.getDouble(0.0);

    if(tv <= 0) {
      SmartDashboard.putBoolean("valid limelight target", false);
      // rightPIDController.setReference(0, ControlType.kVelocity);
      return;
    }

    SmartDashboard.putBoolean("valid limelight target", true);
    
    tx = tyEntry.getDouble(0);
    ty = tyEntry.getDouble(0);

    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putNumber("ty", ty);

    cameraAndTapeAngleDeltaDegrees = ty;
    distance = (tapeHeight - cameraHeight) / Math.tan(degToRad(cameraAngleDegrees) + degToRad(cameraAndTapeAngleDeltaDegrees));

    SmartDashboard.putNumber("distance (m)", distance);

    rpm = distance * 350;

    SmartDashboard.putNumber("calc flywheel rpm", rpm);
    rightPIDController.setReference(rpm, ControlType.kVelocity);
    SmartDashboard.putNumber("actual flywheel rpm", rightMotorEncoder.getVelocity());
  }

  public void rotateShooter() {
    tx = txEntry.getDouble(0.0);
    shooterPosition = Math.min(Math.max(shooterPosition + tx, Constants.ShooterPid.MIN_TURRET_POS),
        Constants.ShooterPid.MAX_TURRET_POS);
    rotatePIDController.setReference((shooterPosition / 360), ControlType.kPosition);
  }

  public void setMotors(double set) {
    SmartDashboard.putNumber("Shooter Speed", set);
    shooterLeftMotor.set(-set);
    shooterRightMotor.set(set);
  }

  private double degToRad(double degrees) {
    return degrees * Math.PI / 180;
  }

  @Override
  public void periodic() {
    fernCalculateFlywheelRpm();
    // double move = m_stick.getRawAxis(Constants.RIGHT_TRIGGER_AXIS);
    // setMotors(Math.abs(move) < Constants.JOYSTICK_DRIFT_TOLERANCE ? 0 : move);
    // int pov = m_stick.getPOV();
    // SmartDashboard.putNumber("Joystick POV", pov);
    // if (!m_pressed) {
    //   switch (pov) {
    //     case Constants.Xbox.POV_UP_BUTTON:
    //       m_pressed = true;
    //       m_speed += 10;
    //       break;
    //     case Constants.Xbox.POV_DOWN_BUTTON:
    //       m_pressed = true;
    //       m_speed -= 10;
    //       break;
    //     case Constants.Xbox.POV_LEFT_BUTTON:
    //       m_pressed = true;
    //       m_speed -= 100;
    //       break;
    //     case Constants.Xbox.POV_RIGHT_BUTTON:
    //       m_pressed = true;
    //       m_speed += 100;
    //       break;
    //   }
    // }
    // if (pov == -1)
    //   m_pressed = false;
    // SmartDashboard.putNumber("Shooter Set (actual rpm)", rightMotorEncoder.getVelocity());
    // SmartDashboard.putNumber("Shooter Set (setpoint rpm)", m_speed);
    // setVelocity();
   }
}
