// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  private final CANSparkMax HookMotor = new CANSparkMax(Constants.CanMotorId.HOOK_MOTOR, MotorType.kBrushless);
  private final CANSparkMax LiftMotor = new CANSparkMax(Constants.CanMotorId.BAR_MOTOR, MotorType.kBrushless);
  private SparkMaxPIDController Lift_PID, Hook_PID;

  enum StepDir {
    kForward,
    kBackward
  }

  private int currentStep = 0;
  private StepDir direction = StepDir.kForward;
  private double sp_Lift = 0;
  private double sp_Hook = 0;
  private double upper_sp_Hook = 0;
  private double lower_sp_Hook = 0;
  private double upper_sp_Lift = 0;
  private double lower_sp_Lift = 0;
  private double Climb_speed = 0;
  private boolean hookSetpointAchieved = false;
  private boolean liftSetpointAchieved = false;
  private double Hook_pos_mm = 0;
  private double Lift_pos_deg = 0;
  private double length_adjacent = 0;
  private double theta = 0;
  private double Lift_angle_command = 0;

  /** Creates a new Climber. */
  public Climber() {
    initPidAndMotors();
  }

  @Override
  public void periodic() {
    if (currentStep > 0 && currentStep < 3) {
      RobotContainer.m_shooterSubsystem.isClimbingFullRotate = true;
      RobotContainer.m_shooterSubsystem.isClimbingLowRotate = false;
    } else if (currentStep >= 3 && currentStep < 12) {
      RobotContainer.m_shooterSubsystem.isClimbingFullRotate = false;
      RobotContainer.m_shooterSubsystem.isClimbingLowRotate = true;
    } else {
      RobotContainer.m_shooterSubsystem.isClimbingFullRotate = false;
      RobotContainer.m_shooterSubsystem.isClimbingLowRotate = false;
    }

    SmartDashboard.putNumber("climb step", currentStep);
    Calculate_theta();
    Climb_Sequence();
    Climb_Speed_Calc();
  }

  private void Calculate_theta(){
    double hook_flip = 0;
    if (currentStep>=7 && currentStep<=10) hook_flip = 356 - HookMotor.getEncoder().getPosition();
    else hook_flip = HookMotor.getEncoder().getPosition();

    // Hook Position = Encoder position * (total travel in mm:850) / (NEO rotations:356)
    Hook_pos_mm = 725.0 - (hook_flip * (850.0/356.0));

    // Lift Position = Encoder position * (total travel in degrees:135) / (NEO rotations:130)
    Lift_pos_deg = -(LiftMotor.getEncoder().getPosition())*(135.0/130.0);

    // Trigonometry to find length (mm) of adjacent side of triangle (32.7 degrees - climb rung angle)
    length_adjacent = Math.acos(Math.toRadians(32.7)) * Hook_pos_mm;

    // Trigonometry to find angle (degrees) of lower triangle to C of G
    theta = Math.toDegrees(Math.acos(length_adjacent/488.0)) + 10;

    if ((theta > 135.0) && (theta < 360.0)) Lift_angle_command = 135.0;
    else if (Double.isNaN(theta)) Lift_angle_command = 45.0;
    else Lift_angle_command = theta;
  }

  private void Climb_Speed_Calc(){
    double rightTriggerVal = RobotContainer.driveJoystick.getRawAxis(Constants.Xbox.RIGHT_TRIGGER_AXIS);
    double leftTriggerVal = RobotContainer.driveJoystick.getRawAxis(Constants.Xbox.LEFT_TRIGGER_AXIS);

    if (rightTriggerVal>0.05) {
      Climb_speed = rightTriggerVal;
      if (direction == StepDir.kBackward) { direction = StepDir.kForward; currentStep++; }
    }
    else if (leftTriggerVal>0.05) {
      Climb_speed = leftTriggerVal;
      if (direction == StepDir.kForward) { direction = StepDir.kBackward; currentStep--; }
    }
    else Climb_speed=0.0;

    Hook_PID.setSmartMotionMaxVelocity(Climb_speed*18000, 0);
    Lift_PID.setSmartMotionMaxVelocity(Climb_speed*18000, 0);

    Hook_PID.setReference(sp_Hook, CANSparkMax.ControlType.kSmartMotion);
    Lift_PID.setReference(sp_Lift, CANSparkMax.ControlType.kSmartMotion);
  }

  private void Climb_Sequence(){
    switch(currentStep) { // Hook(0:356) Lift(0:-130)
      case 0: sp_Hook=0.0; sp_Lift=LiftMotor.getEncoder().getPosition(); break; // Send Hooks HOME (0)
      case 1: sp_Hook=0.0; sp_Lift=0.0; break; // Send Hooks & Lift HOME (0)
      case 2: sp_Hook=0.0; sp_Lift=-133; break; // Lift up to MID rung
      case 3: sp_Hook=80.0; sp_Lift=-133.0; break; // Hooks pull robot up off ground
      case 4: sp_Hook=355.0; sp_Lift=-Lift_angle_command; break; // Move under HIGH rung
      case 5: sp_Hook=355.0; sp_Lift=-132.0; break; // Move up against HIGH rung
      case 6: sp_Hook=330.0; sp_Lift=-133.0; break; // Move back to double hook
      case 7: sp_Hook=330.0; sp_Lift=-Lift_angle_command+6; break; // **Swing body
      case 8: sp_Hook=0.0; sp_Lift=-Lift_angle_command+6; break; // Move under TRAVERSAL rung
      case 9: sp_Hook=0.0; sp_Lift=-133.0; break; // Move up against TRAVERSAL rung
      case 10: sp_Hook=25.0; sp_Lift=-133.0; break; // Move back to double hook
      case 11: sp_Hook=25.0; sp_Lift=-Lift_angle_command; break; // **Swing body
      case 12: sp_Hook=150.0; sp_Lift=-Lift_angle_command; break; // move up
      case 13: sp_Hook=15.0; sp_Lift=-110; break; // move up
    }

    upper_sp_Hook = sp_Hook + 0.3;
    lower_sp_Hook = sp_Hook - 0.3;

    upper_sp_Lift = sp_Lift + 0.9;
    lower_sp_Lift = sp_Lift - 0.9;

    hookSetpointAchieved = (HookMotor.getEncoder().getPosition() <= upper_sp_Hook) && (HookMotor.getEncoder().getPosition() >= lower_sp_Hook);

    liftSetpointAchieved = (LiftMotor.getEncoder().getPosition() <= upper_sp_Lift) && (LiftMotor.getEncoder().getPosition() >= lower_sp_Lift);

    if (hookSetpointAchieved && liftSetpointAchieved) {
      if (RobotContainer.driveJoystick.getRawAxis(Constants.Xbox.RIGHT_TRIGGER_AXIS)>0.05  && (currentStep<13) && (direction == StepDir.kForward)) currentStep+=1;
      if (RobotContainer.driveJoystick.getRawAxis(Constants.Xbox.LEFT_TRIGGER_AXIS)>0.05  && (currentStep>0) && (direction == StepDir.kBackward)) currentStep-=1;
    }
  }

  // this could go in the constructor
  private void initPidAndMotors() {
    Hook_PID = HookMotor.getPIDController();
    Hook_PID.setP(0.00005);
    Hook_PID.setI(0.000000001);
    Hook_PID.setD(0.0000005);
    Hook_PID.setIZone(0);
    Hook_PID.setFF(0.000156);
    Hook_PID.setOutputRange(-1.0, 1.0);
    Hook_PID.setSmartMotionMaxVelocity(500, 0);
    Hook_PID.setSmartMotionMaxAccel(15000, 0);

    Lift_PID = LiftMotor.getPIDController();
    Lift_PID.setP(0.00005);
    Lift_PID.setI(0.000000001);
    Lift_PID.setD(0.0000005);
    Lift_PID.setIZone(0);
    Lift_PID.setFF(0.000156);
    Lift_PID.setOutputRange(-1.0, 1.0);
    Lift_PID.setSmartMotionMaxVelocity(500, 0);
    Lift_PID.setSmartMotionMaxAccel(15000, 0);

    // HOOK Spark Max Default Values
    HookMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    HookMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    HookMotor.setSoftLimit(SoftLimitDirection.kForward, 356);
    HookMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    HookMotor.setIdleMode(IdleMode.kCoast);
    HookMotor.setSmartCurrentLimit(40);
    HookMotor.setInverted(false);
   
    // LIFT Spark Max Default Values
    LiftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    LiftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    LiftMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    LiftMotor.setSoftLimit(SoftLimitDirection.kReverse, -135);
    LiftMotor.setIdleMode(IdleMode.kCoast);
    LiftMotor.setSmartCurrentLimit(40);
    LiftMotor.setInverted(false);
  }

  public double getLiftPositionDeg() {
    return Lift_pos_deg;
  }

}
