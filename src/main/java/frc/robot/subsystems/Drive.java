// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorID.CAN;

public class Drive extends SubsystemBase {
  // motors
  private final CANSparkMax leftFrontMotor =
      new CANSparkMax(CAN.DRIVE_LEFT_FRONT, MotorType.kBrushless);
  private final CANSparkMax leftRearMotor =
      new CANSparkMax(CAN.DRIVE_LEFT_REAR, MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor =
      new CANSparkMax(CAN.DRIVE_RIGHT_FRONT, MotorType.kBrushless);
  private final CANSparkMax rightRearMotor =
      new CANSparkMax(CAN.DRIVE_RIGHT_REAR, MotorType.kBrushless);
  // encoders
  private RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
  private RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

  /** Creates a new Drive. */
  public Drive() {
    leftFrontMotor.setInverted(true);
    leftRearMotor.setInverted(true);
    rightFrontMotor.setInverted(false);
    rightRearMotor.setInverted(false);

    leftRearMotor.follow(leftFrontMotor, false);
    rightRearMotor.follow(rightFrontMotor, false);

    leftEncoder.setPositionConversionFactor(0.4780 / 10.71);
    rightEncoder.setPositionConversionFactor(0.4780 / 10.71);
    leftEncoder.setVelocityConversionFactor(0.4780 / 10.71);
    rightEncoder.setVelocityConversionFactor(0.4780 / 10.71);

    resetEncoders();
    setBrake(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Drive Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Left Drive Velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Drive Position", rightEncoder.getPosition());
    SmartDashboard.putNumber("Right Drive Velocity", rightEncoder.getVelocity());
  }

  public void manualDrive(double xAxisSpeed, double yAxisSpeed) {
    final double driveX = Math.pow(xAxisSpeed, 3) * Constants.Drive.TURNING_SPEED_MULTIPLIER;
    final double driveY = Math.pow(yAxisSpeed, 3) * Constants.Drive.LONGITUDINAL_SPEED_MULTIPLIER;
    leftFrontMotor.set((driveY - driveX) * Constants.Drive.OVERALL_SPEED_MULTIPLIER);
    rightFrontMotor.set((driveY + driveX) * Constants.Drive.OVERALL_SPEED_MULTIPLIER);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
  }

  public void setBrake(boolean isBrake) {
    if (isBrake) {
      rightFrontMotor.setIdleMode(IdleMode.kBrake);
      rightRearMotor.setIdleMode(IdleMode.kBrake);
      leftRearMotor.setIdleMode(IdleMode.kBrake);
      leftFrontMotor.setIdleMode(IdleMode.kBrake);
    } else {
      rightFrontMotor.setIdleMode(IdleMode.kCoast);
      rightRearMotor.setIdleMode(IdleMode.kCoast);
      leftRearMotor.setIdleMode(IdleMode.kCoast);
      leftFrontMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void stopMotors() {
    leftFrontMotor.stopMotor();
    rightFrontMotor.stopMotor();
  }
}
