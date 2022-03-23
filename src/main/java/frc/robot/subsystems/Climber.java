// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.MP.Loop;
import frc.utils.Logger;

@Deprecated
public class Climber extends SubsystemBase {

  public final CANSparkMax Hook = new CANSparkMax(Constants.CanMotorId.HOOK_MOTOR, MotorType.kBrushless);
  public final CANSparkMax Lift = new CANSparkMax(Constants.CanMotorId.BAR_MOTOR, MotorType.kBrushless);
  private SparkMaxPIDController Lift_PID, Hook_PID;
  private final double ARM_RATIO = 70; // gear ratio * gearbox ratio
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, m_epsilon;
  private Joystick m_stick;
  private double armPosition = 0;
  private ArmKinematics armKinematics;
  private boolean isManualControl = true;

  int climb_step = 0;
  int Step_dir = 1;
  double sp_Lift = 0;
  double sp_Hook = 0;
  double upper_sp_Hook = 0;
  double lower_sp_Hook = 0;
  double upper_sp_Lift = 0;
  double lower_sp_Lift = 0;
  double Climb_speed = 0;
  int sp_Hook_achieved = 0;
  int sp_Lift_achieved = 0;
  double Hook_pos_mm = 0;
  double Lift_pos_deg = 0;
  double length_adjacent = 0;
  double theta = 0;
  double Lift_angle_command = 0;

  /** Creates a new Climber. */
  public Climber() {
    resetEncoder();
    initPidAndMotors();
  }

  @Override
  public void periodic() {
    sequence();
    arm();
  }

  public void Calculate_theta(){
    double hook_flip = 0;
    if (climb_step>=7 && climb_step<=10) hook_flip = 356 - Hook.getEncoder().getPosition();
    else hook_flip = Hook.getEncoder().getPosition();

    // Hook Position = Encoder position * (total travel in mm:850) / (NEO rotations:356)
    Hook_pos_mm = 725.0 - (hook_flip * (850.0/356.0));

    // Lift Position = Encoder position * (total travel in degrees:135) / (NEO rotations:130)
    Lift_pos_deg = -(Lift.getEncoder().getPosition())*(135.0/130.0);

    // Trigonometry to find length (mm) of adjacent side of triangle (32.7 degrees - climb rung angle)
    length_adjacent = Math.acos(Math.toRadians(32.7)) * Hook_pos_mm;

    // Trigonometry to find angle (degrees) of lower triangle to C of G
    theta = Math.toDegrees(Math.acos(length_adjacent/488.0)) + 10;

    if ((theta > 135.0) && (theta < 360.0)) Lift_angle_command = 135.0;
    else if (Double.isNaN(theta)) Lift_angle_command = 45.0;
    else Lift_angle_command = theta;
  }

  public void arm(){
    double rightTriggerVal = RobotContainer.driveJoystick.getRawAxis(Constants.Xbox.RIGHT_TRIGGER_AXIS);
    double leftTriggerVal = RobotContainer.driveJoystick.getRawAxis(Constants.Xbox.LEFT_TRIGGER_AXIS);

    if (rightTriggerVal>0.05) {
      Climb_speed = rightTriggerVal;
      if (Step_dir!=1) { Step_dir=1; climb_step+=1; }
    }
    else if (leftTriggerVal>0.05) {
      Climb_speed = leftTriggerVal;
      if (Step_dir!=0) { Step_dir=0; climb_step-=1; }
    }
    else Climb_speed=0.0;

    Hook_PID.setSmartMotionMaxVelocity(Climb_speed*12000, 0);
    Lift_PID.setSmartMotionMaxVelocity(Climb_speed*12000, 0);

    Hook_PID.setReference(sp_Hook, CANSparkMax.ControlType.kSmartMotion);
    Lift_PID.setReference(sp_Lift, CANSparkMax.ControlType.kSmartMotion);
  }

  public void sequence(){
    switch(climb_step) { // Hook(0:356) Lift(0:-130)
      case 0: sp_Hook=0.0; sp_Lift=Lift.getEncoder().getPosition(); break; // Send Hooks HOME (0)
      case 1: sp_Hook=0.0; sp_Lift=0.0; break; // Send Hooks & Lift HOME (0)
      case 2: sp_Hook=0.0; sp_Lift=-130; break; // Lift up to MID rung
      case 3: sp_Hook=80.0; sp_Lift=-130.0; break; // Hooks pull robot up off ground
      case 4: sp_Hook=355.0; sp_Lift=-Lift_angle_command; break; // Move under HIGH rung
      case 5: sp_Hook=355.0; sp_Lift=-130.0; break; // Move up against HIGH rung
      case 6: sp_Hook=330.0; sp_Lift=-130.0; break; // Move back to double hook
      case 7: sp_Hook=330.0; sp_Lift=-Lift_angle_command; break; // **Swing body
      case 8: sp_Hook=0.0; sp_Lift=-Lift_angle_command; break; // Move under TRAVERSAL rung
      case 9: sp_Hook=0.0; sp_Lift=-130.0; break; // Move up against TRAVERSAL rung
      case 10: sp_Hook=25.0; sp_Lift=-130.0; break; // Move back to double hook
      case 11: sp_Hook=25.0; sp_Lift=-Lift_angle_command; break; // **Swing body
      case 12: sp_Hook=150.0; sp_Lift=-Lift_angle_command; break; // move up
    }

    upper_sp_Hook = sp_Hook + 0.2;
    lower_sp_Hook = sp_Hook - 0.2;

    upper_sp_Lift = sp_Lift + 0.8;
    lower_sp_Lift = sp_Lift - 0.8;

    if ((Hook.getEncoder().getPosition() <= upper_sp_Hook) && (Hook.getEncoder().getPosition() >= lower_sp_Hook)) sp_Hook_achieved = 1;
    else sp_Hook_achieved = 0;

    if ((Lift.getEncoder().getPosition() <= upper_sp_Lift) && (Lift.getEncoder().getPosition() >= lower_sp_Lift)) sp_Lift_achieved = 1;
    else sp_Lift_achieved = 0;

    if (sp_Hook_achieved==1 && sp_Lift_achieved==1) {
      if (RobotContainer.driveJoystick.getRawAxis(Constants.Xbox.RIGHT_TRIGGER_AXIS)>0.05  && (climb_step<12) && (Step_dir==1)) climb_step+=1;
      if (RobotContainer.driveJoystick.getRawAxis(Constants.Xbox.LEFT_TRIGGER_AXIS)>0.05  && (climb_step>0) && (Step_dir==0)) climb_step-=1;
    }
  }

  private void initPidAndMotors() {
    Hook_PID = Hook.getPIDController();
    Hook_PID.setP(0.00005);
    Hook_PID.setI(0.000000001);
    Hook_PID.setD(0.0000005);
    Hook_PID.setIZone(0);
    Hook_PID.setFF(0.000156);
    Hook_PID.setOutputRange(-1.0, 1.0);
    Hook_PID.setSmartMotionMaxVelocity(500, 0);
    Hook_PID.setSmartMotionMaxAccel(15000, 0);

    Lift_PID = Lift.getPIDController();
    Lift_PID.setP(0.00005);
    Lift_PID.setI(0.000000001);
    Lift_PID.setD(0.0000005);
    Lift_PID.setIZone(0);
    Lift_PID.setFF(0.000156);
    Lift_PID.setOutputRange(-1.0, 1.0);
    Lift_PID.setSmartMotionMaxVelocity(500, 0);
    Lift_PID.setSmartMotionMaxAccel(15000, 0);

    // HOOK Spark Max Default Values
    Hook.enableSoftLimit(SoftLimitDirection.kForward, true);
    Hook.enableSoftLimit(SoftLimitDirection.kReverse, true);
    Hook.setSoftLimit(SoftLimitDirection.kForward, 356);
    Hook.setSoftLimit(SoftLimitDirection.kReverse, 0);
    Hook.setIdleMode(IdleMode.kCoast);
    Hook.setSmartCurrentLimit(40);
    Hook.setInverted(false);
   
    // LIFT Spark Max Default Values
    Lift.enableSoftLimit(SoftLimitDirection.kForward, true);
    Lift.enableSoftLimit(SoftLimitDirection.kReverse, true);
    Lift.setSoftLimit(SoftLimitDirection.kForward, 0);
    Lift.setSoftLimit(SoftLimitDirection.kReverse, -135);
    Lift.setIdleMode(IdleMode.kCoast);
    Lift.setSmartCurrentLimit(40);
    Lift.setInverted(false);
  }

  public void setArmVolt(double percent) {
    Lift.set(percent);
  }

  public void manualBarMotorControl() {
    double rawAxisValue = m_stick.getRawAxis(Constants.Xbox.RIGHT_STICK_Y_AXIS);
    double inc = (Math.abs(rawAxisValue) < Constants.Xbox.JOYSTICK_DRIFT_TOLERANCE ? 0 : rawAxisValue)
        * Constants.ArmValue.ARM_SPEED_MULTIPLIER;
    // armPosition = Math.min(Math.max(armPosition + inc, Constants.MIN_ARM_POS),
    // Constants.MAX_ARM_POS);
    armPosition = Math.min(armPosition + inc, Constants.ArmValue.MAX_ARM_POS);

    setArmPosition(armPosition);

    SmartDashboard.putNumber("Arm Position", armPosition);
  }

  public void manualHookMotorControl() {
    double inc = m_stick.getRawAxis(Constants.Xbox.RIGHT_TRIGGER_AXIS) - m_stick.getRawAxis(Constants.Xbox.LEFT_TRIGGER_AXIS);
    setHook(inc * Constants.ArmValue.HOOK_SPEED_MULTIPLIER);
  }

  public boolean isArmInPosition(double desiredPosition) {
    return Math.abs(getArmPosition() - desiredPosition) < m_epsilon;
  }

  public boolean isHookInPosition(double desiredPosition) {
    return Math.abs(getHookPosition() - desiredPosition) < m_epsilon;
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
    Lift_PID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    Lift_PID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    Lift_PID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    Lift_PID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    double armSetp = degToMotorPos(position);
    // System.out.println(armSetp);
    Lift_PID.setReference(armSetp, ControlType.kSmartMotion);
  }

  public void setHookPosition(double position) {
    kP = SmartDashboard.getNumber("Hook P Gain", 0);
    kI = SmartDashboard.getNumber("Hook I Gain", 0);
    kD = SmartDashboard.getNumber("Hook D Gain", 0);
    kIz = SmartDashboard.getNumber("Hook I Zone", 0);
    kFF = SmartDashboard.getNumber("Hook Feed Forward", 0);
    kMaxOutput = SmartDashboard.getNumber("Hook Max Output", 0);
    kMinOutput = SmartDashboard.getNumber("Hook Min Output", 0);
    maxVel = SmartDashboard.getNumber("Hook Max Velocity", 0);
    minVel = SmartDashboard.getNumber("Hook Min Velocity", 0);
    maxAcc = SmartDashboard.getNumber("Hook Max Acceleration", 0);
    allowedErr = SmartDashboard.getNumber("Hook Allowed Closed Loop Error", 0);
    m_epsilon = SmartDashboard.getNumber("Hook epsilon for float compare", 0.01);

    // int smartMotionSlot = 0;
    // hookPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    // hookPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    // hookPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    // hookPID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // System.out.println(armSetp);
    Hook_PID.setReference(position, ControlType.kSmartMotion);
  }

  public void disableArm() {
    Lift.set(0);
  }

  public void disableHook() {
    Hook.set(0);
  }

  public void setArmToZero() {
    armPosition = 0;
  }

  public void setManualClimb(boolean isManualControl) {
    this.isManualControl = isManualControl;
  }

  public void toggleManClimb() {
    this.isManualControl = !this.isManualControl;
  }

  private double degToMotorPos(double degrees) {
    return (degrees / 360) * ARM_RATIO;
  }

  private double motorPosToDeg(double position) {
    return 360 * position / (ARM_RATIO);
  }

  public void resetEncoder() {
    Lift.getEncoder().setPosition(0);
    Hook.getEncoder().setPosition(0);
  }

  public double getArmPosition() {
    return motorPosToDeg(Lift.getEncoder().getPosition());
  }

  public double getHookPosition() {
    return Hook.getEncoder().getPosition();
  }

  private void initValues() {
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
    Lift_PID.setP(kP);
    Lift_PID.setI(kI);
    Lift_PID.setD(kD);
    Lift_PID.setIZone(kIz);
    Lift_PID.setFF(kFF);
    Lift_PID.setOutputRange(kMinOutput, kMaxOutput);

    Hook_PID.setP(kP);
    Hook_PID.setI(kI);
    Hook_PID.setD(kD);
    Hook_PID.setIZone(kIz);
    Hook_PID.setFF(kFF);
    Hook_PID.setOutputRange(kMinOutput, kMaxOutput);

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
    Lift_PID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    Lift_PID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    Lift_PID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    Lift_PID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    Hook_PID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    Hook_PID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    Hook_PID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    Hook_PID.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

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

    // display float compare epslion
    SmartDashboard.putNumber("epsilon for float compare", m_epsilon);
  }

  public void setHook(double power) {
    Hook.set(power);
  }

  public void resetHook() {
    Hook.getEncoder().setPosition(0);
  }

  public ArmKinematics getArmKinematics() {
    return this.armKinematics;
  }
}
