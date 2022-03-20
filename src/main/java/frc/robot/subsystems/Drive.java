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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.MP.Loop;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  private final CANSparkMax m_leftFrontMotor = new CANSparkMax(Constants.CanMotorId.LEFT_FRONT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_leftRearMotor = new CANSparkMax(Constants.CanMotorId.LEFT_REAR_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_rightFrontMotor = new CANSparkMax(Constants.CanMotorId.RIGHT_FRONT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_rightRearMotor = new CANSparkMax(Constants.CanMotorId.RIGHT_REAR_MOTOR, MotorType.kBrushless);
  private RelativeEncoder leftEncoder = m_leftFrontMotor.getEncoder();
  private RelativeEncoder rightEncoder = m_rightFrontMotor.getEncoder();
  private Joystick m_driveJoystick;
  AHRS bodyNavx;
  private final DifferentialDriveOdometry m_odometry;
  public boolean isReverse = false;

  private final Loop mLoop = new Loop() {
    @Override
    public void onStart() {
      System.out.println("Starting Climber loop");
      // logger.openFile();
    }

    @Override
    public void onLoop() {
      // logger.logTele();
    }

    @Override
    public void onStop() {
      System.out.println("Ending Climber loop");
      // logger.close();
    }

  };

  public Drive() {
    m_driveJoystick = RobotContainer.driveJoystick;
    this.bodyNavx = RobotContainer.bodyNavx;
    m_odometry = new DifferentialDriveOdometry(bodyNavx.getRotation2d());
    leftEncoder.setPositionConversionFactor(0.4780/10.71);
    rightEncoder.setPositionConversionFactor(0.4780/10.71);
    // leftEncoder.setPositionConversionFactor(1);
    leftEncoder.setVelocityConversionFactor(0.4780/10.71);
    rightEncoder.setVelocityConversionFactor(0.4780/10.71);
    // rightEncoder.setPositionConversionFactor(1);
    // 10.71*0.4780
    m_leftFrontMotor.setInverted(true);
    m_leftRearMotor.setInverted(true);
  }

  public void updateOdometry(){
    if(isReverse){
    m_odometry.update(bodyNavx.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }else{
    m_odometry.update(bodyNavx.getRotation2d().unaryMinus(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }
    SmartDashboard.putNumber("posex", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("posey", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("deg", m_odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("velleft", getWheelSpeeds().leftMetersPerSecond);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double driveX = Math.pow(m_driveJoystick.getRawAxis(Constants.Xbox.LEFT_STICK_X_AXIS), 3);
    // double driveY = Math.pow(m_driveJoystick.getRawAxis(Constants.Xbox.LEFT_STICK_Y_AXIS), 3);
    // double leftMotors = driveY - driveX;
    // double rightMotors = driveY + driveX;

    updateOdometry();
    

    SmartDashboard.putNumber("posex", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("posey", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("deg", m_odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("velleft", getWheelSpeeds().leftMetersPerSecond);

    
    SmartDashboard.putNumber("getpos", leftEncoder.getPosition());
    // tankDriveVolts(3, 3);
    // setMotors(leftMotors, rightMotors);
  }

  public void setMotors(double left, double right) {
    m_leftFrontMotor.set(left);
    m_leftRearMotor.set(left);
    m_rightFrontMotor.set(right);
    m_rightRearMotor.set(right);
  }

  public void manualDrive(double xAxisSpeed, double yAxisSpeed) {
    double driveX = Math.pow(xAxisSpeed, 3);
    double driveY = Math.pow(yAxisSpeed, 3);
    double leftMotors = driveY - driveX;
    double rightMotors = driveY + driveX;
    setMotors(leftMotors*0.7, rightMotors*0.7);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    if(isReverse){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity()/60, rightEncoder.getVelocity()/60);
    }
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity()/60, rightEncoder.getVelocity()/60);
  }

  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(-RobotContainer.bodyNavx.getYaw()));
  }
  
  public void setBreak(boolean isBreak){
    if (isBreak){
      m_rightFrontMotor.setIdleMode(IdleMode.kBrake);
      m_rightRearMotor.setIdleMode(IdleMode.kBrake);
      m_leftRearMotor.setIdleMode(IdleMode.kBrake);
      m_leftFrontMotor.setIdleMode(IdleMode.kBrake);
    }else{
      m_rightFrontMotor.setIdleMode(IdleMode.kCoast);
      m_rightRearMotor.setIdleMode(IdleMode.kCoast);
      m_leftRearMotor.setIdleMode(IdleMode.kCoast);
      m_leftFrontMotor.setIdleMode(IdleMode.kCoast);

    }
  }
  public void tankDriveVolts(double left, double right){
    m_leftFrontMotor.setVoltage(left);
    m_leftRearMotor.setVoltage(left);
    m_rightFrontMotor.setVoltage(right);
    m_rightRearMotor.setVoltage(right);
  }

  public void tankDriveVoltsReverse(double left, double right){
    m_leftFrontMotor.setVoltage(-right);
    m_leftRearMotor.setVoltage(-right);
    m_rightFrontMotor.setVoltage(-left);
    m_rightRearMotor.setVoltage(-left);
  }
}
