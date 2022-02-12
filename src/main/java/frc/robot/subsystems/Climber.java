// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final CANSparkMax hookMotor = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax barMotor = new CANSparkMax(5, MotorType.kBrushless);
  private SparkMaxPIDController barPID;
  private final double ARMRATIO = 1/70; 
  public Climber() {
    resetEncoder();
    barPID = barMotor.getPIDController();
    barPID.setP(0.034173);
    barPID.setI(0.0001);
    barPID.setOutputRange(-1, 0.1);
    
    hookMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmPosition(double position){
    double armSetp = degToMotorPos(position); 
    barPID.setReference(armSetp, ControlType.kPosition);
  }

  public void disableArm(){
    barMotor.set(0);
  }

  private double degToMotorPos(double degrees){
    return (degrees*ARMRATIO)/360;
  }

  private double motorPosToDeg(double position){
    return 360*position/(ARMRATIO);
  }

  public void resetEncoder(){
    barMotor.getEncoder().setPosition(0);
  }

  public double getPosition(){
    return motorPosToDeg(barMotor.getEncoder().getPosition());
  }
}
