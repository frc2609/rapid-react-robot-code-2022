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
import frc.robot.RobotContainer;
import frc.robot.MP.Loop;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  private final CANSparkMax m_leftFrontMotor = new CANSparkMax(Constants.CanMotorId.LEFT_FRONT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_leftRearMotor = new CANSparkMax(Constants.CanMotorId.LEFT_REAR_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_rightFrontMotor = new CANSparkMax(Constants.CanMotorId.RIGHT_FRONT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_rightRearMotor = new CANSparkMax(Constants.CanMotorId.RIGHT_REAR_MOTOR, MotorType.kBrushless);
  private Joystick m_driveJoystick;
  AHRS bodyNavx;
  private final DifferentialDriveOdometry m_odometry;

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double driveX = Math.pow(m_driveJoystick.getRawAxis(Constants.Xbox.LEFT_STICK_X_AXIS), 3);
    double driveY = Math.pow(m_driveJoystick.getRawAxis(Constants.Xbox.LEFT_STICK_Y_AXIS), 3);
    double leftMotors = driveY - driveX;
    double rightMotors = driveY + driveX;

    m_odometry.update(bodyNavx.getRotation2d(), m_leftFrontMotor.getEncoder().getPosition(), m_rightFrontMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("posex", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("posey", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("deg", m_odometry.getPoseMeters().getRotation().getDegrees());
    // setMotors(leftMotors, rightMotors);
  }

  private void setMotors(double left, double right) {
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
}
