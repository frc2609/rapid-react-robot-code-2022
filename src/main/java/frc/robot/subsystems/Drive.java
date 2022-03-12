// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
//import frc.robot.RobotContainer;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  private final CANSparkMax m_leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_leftRearMotor = new CANSparkMax(Constants.LEFT_REAR_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_rightRearMotor = new CANSparkMax(Constants.RIGHT_REAR_MOTOR, MotorType.kBrushless);
  private Joystick m_driveJoystick;
  AHRS bodyNavx;
  private final DifferentialDriveOdometry m_odometry;


  public Drive(Joystick driveJoystick, AHRS bodyNavx) {
    m_driveJoystick = driveJoystick;
    this.bodyNavx = bodyNavx;
    m_odometry = new DifferentialDriveOdometry(bodyNavx.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double driveX = Math.pow(m_driveJoystick.getRawAxis(Constants.LEFT_STICK_X_AXIS), 3);
    double driveY = Math.pow(m_driveJoystick.getRawAxis(Constants.LEFT_STICK_Y_AXIS), 3);
    double leftMotors = driveY - driveX;
    double rightMotors = driveY + driveX;

    m_odometry.update(bodyNavx.getRotation2d(), m_leftFrontMotor.getEncoder().getPosition(), m_rightFrontMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("posex", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("posey", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("deg", m_odometry.getPoseMeters().getRotation().getDegrees());
    // setMotors(leftMotors, rightMotors);
  }

  private void setMotors(double left, double right) {
    m_leftFrontMotor.set(-left * 0.3);
    m_leftRearMotor.set(-left * 0.3);
    m_rightFrontMotor.set(right * 0.3);
    m_rightRearMotor.set(right * 0.3);
  }
}
