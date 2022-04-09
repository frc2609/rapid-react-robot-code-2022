// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.filter.LinearFilter;

public class Drive extends SubsystemBase {
  // motors
  private final CANSparkMax m_leftFrontMotor = new CANSparkMax(Constants.CanMotorId.LEFT_FRONT_MOTOR,
      MotorType.kBrushless);
  public static final CANSparkMax m_leftRearMotor = new CANSparkMax(Constants.CanMotorId.LEFT_REAR_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax m_rightFrontMotor = new CANSparkMax(Constants.CanMotorId.RIGHT_FRONT_MOTOR,
      MotorType.kBrushless);
  private final CANSparkMax m_rightRearMotor = new CANSparkMax(Constants.CanMotorId.RIGHT_REAR_MOTOR,
      MotorType.kBrushless);
  // encoders
  private RelativeEncoder leftEncoder = m_leftFrontMotor.getEncoder();
  private RelativeEncoder rightEncoder = m_rightFrontMotor.getEncoder();
  // filters
  public LinearFilter yJoystickFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
  public LinearFilter xJoystickFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  // auto
  private final DifferentialDriveOdometry m_odometry;
  // navx
  public AHRS bodyNavx;
  // other
  private boolean isReverse = false;
  private boolean isDriveLocked = false;

  /** Creates a new Drive. */
  public Drive() {
    this.bodyNavx = RobotContainer.bodyNavx;
    m_odometry = new DifferentialDriveOdometry(bodyNavx.getRotation2d());

    m_leftFrontMotor.setInverted(true);
    m_leftRearMotor.setInverted(true);
  
    leftEncoder.setPositionConversionFactor(0.4780 / 10.71);
    rightEncoder.setPositionConversionFactor(0.4780 / 10.71);
    leftEncoder.setVelocityConversionFactor(0.4780 / 10.71);
    rightEncoder.setVelocityConversionFactor(0.4780 / 10.71);
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public void setMotors(double left, double right) {
    m_leftFrontMotor.set(left);
    m_leftRearMotor.set(left);
    m_rightFrontMotor.set(right);
    m_rightRearMotor.set(right);
  }

  public boolean isDrivingForward() {
    return (
      m_leftRearMotor.getEncoder().getVelocity() < Constants.Drive.isDrivingForwardDeadzone
      && m_rightRearMotor.getEncoder().getVelocity() < Constants.Drive.isDrivingForwardDeadzone
    );
  }

  // constant turning speed even at high speeds
  public void manualDrive(double xAxisSpeed, double yAxisSpeed) {
    double driveX = xJoystickFilter.calculate(Math.pow(xAxisSpeed, 3));
    double driveY = Math.pow(yAxisSpeed, 3);
    boolean atRiskForTippingReverse = isDrivingForward() && yAxisSpeed > Constants.Xbox.JOYSTICK_DRIFT_TOLERANCE;

    if (atRiskForTippingReverse) {
      driveY *= 0.22;
    }

    driveY = yJoystickFilter.calculate(driveY);

    double leftMotorPower = driveY - driveX;
    double rightMotorPower = driveY + driveX;

    if (!isDriveLocked) {
        setMotors(leftMotorPower, rightMotorPower);
    }
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    if (isReverse) {
      return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity() / 60, rightEncoder.getVelocity() / 60);
    }
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity() / 60, rightEncoder.getVelocity() / 60);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(-RobotContainer.bodyNavx.getYaw()));
  }

  public void setBrake(boolean isBrake) {
    if (isBrake) {
      m_rightFrontMotor.setIdleMode(IdleMode.kBrake);
      m_rightRearMotor.setIdleMode(IdleMode.kBrake);
      m_leftRearMotor.setIdleMode(IdleMode.kBrake);
      m_leftFrontMotor.setIdleMode(IdleMode.kBrake);
    } else {
      m_rightFrontMotor.setIdleMode(IdleMode.kCoast);
      m_rightRearMotor.setIdleMode(IdleMode.kCoast);
      m_leftRearMotor.setIdleMode(IdleMode.kCoast);
      m_leftFrontMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void tankDriveVolts(double left, double right) {
    m_leftFrontMotor.setVoltage(left);
    m_leftRearMotor.setVoltage(left);
    m_rightFrontMotor.setVoltage(right);
    m_rightRearMotor.setVoltage(right);
  }

  public double getLeftMotorPosition() { return m_leftFrontMotor.getEncoder().getPosition(); }
  public double getRightMotorPosition() { return m_rightFrontMotor.getEncoder().getPosition(); }
  public Pose2d getPose() { return m_odometry.getPoseMeters(); }
  public void setReverse(boolean on) { isReverse = on; }
  public void setDriveLock(boolean on) { isDriveLocked = on; }

  private void updateOdometry() {
    if (isReverse) {
      m_odometry.update(bodyNavx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    } else {
      m_odometry.update(bodyNavx.getRotation2d().unaryMinus(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }
    // SmartDashboard.putNumber("posex", m_odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("posey", m_odometry.getPoseMeters().getY());
    // SmartDashboard.putNumber("deg", m_odometry.getPoseMeters().getRotation().getDegrees());
    // SmartDashboard.putNumber("velleft", getWheelSpeeds().leftMetersPerSecond);
  }

  private void logDriveData() {
    SmartDashboard.putNumber("left front motor current", m_leftFrontMotor.getOutputCurrent());
    SmartDashboard.putNumber("left rear motor current", m_leftRearMotor.getOutputCurrent());
    SmartDashboard.putNumber("right front motor current", m_rightFrontMotor.getOutputCurrent());
    SmartDashboard.putNumber("right rear motor current", m_rightRearMotor.getOutputCurrent());

    SmartDashboard.putNumber("left front motor temperature", m_leftFrontMotor.getMotorTemperature());
    SmartDashboard.putNumber("left rear motor temperature", m_leftRearMotor.getMotorTemperature());
    SmartDashboard.putNumber("right front motor temperature", m_rightFrontMotor.getMotorTemperature());
    SmartDashboard.putNumber("right rear motor temperature", m_rightRearMotor.getMotorTemperature());

    SmartDashboard.putNumber("left front motor velocity", m_leftFrontMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("left rear motor velocity", m_leftRearMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("right front motor velocity", m_rightFrontMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("right rear motor velocity", m_rightRearMotor.getEncoder().getVelocity());
  }
}
