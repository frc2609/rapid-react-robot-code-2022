// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotContainer;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */
  
  private final CANSparkMax m_frontLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax m_rearLeftMotor = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax m_frontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_rearRightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private Joystick m_driveJoystick;

  public Drive(Joystick driveJoystick) {
    m_driveJoystick = driveJoystick;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double driveX = Math.pow(m_driveJoystick.getRawAxis(0), 3);
    double driveY = Math.pow(m_driveJoystick.getRawAxis(1), 3);
    double leftMotors = driveY - driveX;
    double rightMotors = driveY + driveX;
    setMotors(leftMotors, rightMotors);
  }

  private void setMotors(double left, double right){
    m_frontLeftMotor.set(left);
    m_rearLeftMotor.set(-left);
    m_frontRightMotor.set(-right);
    m_rearRightMotor.set(right);
  }
}
