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
import frc.robot.Constants;
import frc.robot.Constants.XboxConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final CANSparkMax hookMotor = new CANSparkMax(Constants.HOOK_MOTOR, MotorType.kBrushless);
  private final CANSparkMax barMotor = new CANSparkMax(Constants.BAR_MOTOR, MotorType.kBrushless);
  private SparkMaxPIDController barPID;
  private final double ARM_RATIO = 70; // gear ratio * gearbox ratio
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, m_epsilon;
  private Joystick m_stick;
  private double armPosition = 0;

  public Climber(Joystick stick) {
    resetEncoder();
    barMotor.setInverted(true);
    barPID = barMotor.getPIDController();
    initValues();
    m_stick = stick;
    barMotor.setIdleMode(IdleMode.kBrake);
    hookMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("HookPositionEncoder", getHookPosition());
    SmartDashboard.putNumber("BarPositionAngle", getArmPosition());
    manualBarMotorControl();
    manualHookMotorControl();

    // setHook(-m_stick.getRawAxis(Constants.RIGHT_STICK_Y_AXIS));
  }

  public void manualBarMotorControl() {
    double rawAxisValue = m_stick.getRawAxis(XboxConstants.RIGHT_STICK_Y_AXIS);
    double inc = (Math.abs(rawAxisValue) < XboxConstants.JOYSTICK_DRIFT_TOLERANCE ? 0 : rawAxisValue) * Constants.ARM_SPEED_MULTIPLIER;
    armPosition = Math.min(Math.max(armPosition+inc, Constants.MIN_ARM_POS), Constants.MAX_ARM_POS);
    
    setArmPosition(armPosition);
  }

  public void manualHookMotorControl() {
    double inc = m_stick.getRawAxis(XboxConstants.RIGHT_TRIGGER_AXIS) - m_stick.getRawAxis(XboxConstants.LEFT_TRIGGER_AXIS);
    setHook(inc);
  }

  public boolean isArmInPosition(double desiredPosition) {
    return Math.abs(getArmPosition() - desiredPosition) < m_epsilon;
  }

  public void setArmPosition(double position) {
    kP = SmartDashboard.getNumber("P Gain", 0);
    kI = SmartDashboard.getNumber("I Gain", 0);
    kD = SmartDashboard.getNumber("D Gain", 0);
    kIz = SmartDashboard.getNumber("I Zone", 0);
    kFF = SmartDashboard.getNumber("Feed Forward", 0);
    kMaxOutput = SmartDashboard.getNumber("Max Output", 0);
    kMinOutput = SmartDashboard.getNumber("Min Output", 0);
    maxVel = SmartDashboard.getNumber("Max Velocity", 0);
    minVel = SmartDashboard.getNumber("Min Velocity", 0);
    maxAcc = SmartDashboard.getNumber("Max Acceleration", 0);
    allowedErr = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
    m_epsilon = SmartDashboard.getNumber("epsilon for float compare", 0.01);
    
    int smartMotionSlot = 0;
    barPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    barPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    barPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    barPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    
    double armSetp = degToMotorPos(position); 
    System.out.println(armSetp);
    barPID.setReference(armSetp, ControlType.kSmartMotion);
  }

  public void disableArm(){
    barMotor.set(0);
  }

  public void setArmToZero() {
    armPosition = 0;
  }

  private double degToMotorPos(double degrees){
    return (degrees/360)*ARM_RATIO;
  }

  private double motorPosToDeg(double position){
    return 360*position/(ARM_RATIO);
  }

  public void resetEncoder(){
    barMotor.getEncoder().setPosition(0);
    hookMotor.getEncoder().setPosition(0);
  }

  public double getArmPosition(){
    return motorPosToDeg(barMotor.getEncoder().getPosition());
  }

  public double getHookPosition(){
    return hookMotor.getEncoder().getPosition();
  }
  
  private void initValues(){
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    m_epsilon = 0.01;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    barPID.setP(kP);
    barPID.setI(kI);
    barPID.setD(kD);
    barPID.setIZone(kIz);
    barPID.setFF(kFF);
    barPID.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    barPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    barPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    barPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    barPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    //display float compare epslion
    SmartDashboard.putNumber("epsilon for float compare", m_epsilon);
  }

  public void setHook(double power){
    hookMotor.set(power);
  }

  public void resetHook(){
    hookMotor.getEncoder().setPosition(0);
  }

}
