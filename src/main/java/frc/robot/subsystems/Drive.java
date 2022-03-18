// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private final CANSparkMax m_leftFrontMotor = new CANSparkMax(Constants.CanMotorId.LEFT_FRONT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_leftRearMotor = new CANSparkMax(Constants.CanMotorId.LEFT_REAR_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_rightFrontMotor = new CANSparkMax(Constants.CanMotorId.RIGHT_FRONT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax m_rightRearMotor = new CANSparkMax(Constants.CanMotorId.RIGHT_REAR_MOTOR, MotorType.kBrushless);
  
  public Drive() {}

  @Override
  public void periodic() {}

  public void manualDrive(double xAxisSpeed, double yAxisSpeed) {
    double driveX = Math.pow(xAxisSpeed, 3);
    double driveY = Math.pow(yAxisSpeed, 3);
    double leftMotors = driveY - driveX;
    double rightMotors = driveY + driveX;
    setMotors(leftMotors, rightMotors);
  }

  private void setMotors(double left, double right){
    m_leftFrontMotor.set(-left * 0.7);
    m_leftRearMotor.set(-left * 0.7);
    m_rightFrontMotor.set(right * 0.7);
    m_rightRearMotor.set(right * 0.7);
  }
}
