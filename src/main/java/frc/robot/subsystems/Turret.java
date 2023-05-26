// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PID;
import frc.robot.Constants.MotorID.CAN;

/**
 * Controls turret rotation only. Use {@link Shooter} for shooting balls.
 */
public class Turret extends SubsystemBase {
  private final CANSparkMax rotateMotor = new CANSparkMax(CAN.SHOOTER_ROTATE, MotorType.kBrushless);
  private final RelativeEncoder rotateEncoder = rotateMotor.getEncoder();
  private SparkMaxPIDController rotatePIDController;

  /** Creates a new Turret. */
  public Turret() {
    rotateMotor.setInverted(true);
    rotateMotor.setIdleMode(IdleMode.kBrake);
    rotatePIDController = rotateMotor.getPIDController();

    setPidValues();
    stopMotor();
    resetMotorEncoders();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Rotation Motor Current", rotateMotor.getOutputCurrent());
    SmartDashboard.putNumber("Turret Position", rotateEncoder.getPosition());
  }

  public void resetMotorEncoders() {
    rotateEncoder.setPosition(0.0);
  }

  public void rotateTurret(double set) {
    rotateMotor.set(set * Constants.Shooter.ROTATION_SENSITIVITY_MULTIPLIER);
  }

  private void setPidValues() {
    rotatePIDController.setP(PID.Rotate.PROPORTIONAL);
    rotatePIDController.setI(PID.Rotate.INTEGRAL);
    rotatePIDController.setD(PID.Rotate.DERIVATIVE);
    rotatePIDController.setIZone(PID.Rotate.INTEGRAL);
    rotatePIDController.setFF(PID.Rotate.FEED_FORWARD);
    rotatePIDController.setOutputRange(PID.Rotate.MIN_OUTPUT,
        PID.Rotate.MAX_OUTPUT);
  }

  public void stopMotor() {
    rotateMotor.stopMotor();
  }
}
