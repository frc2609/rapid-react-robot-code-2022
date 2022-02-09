// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; // selecting auto modes
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax; // motors
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // specifying brushless motor type

import com.kauailabs.navx.frc.AHRS; // for NavX Gyro
import edu.wpi.first.wpilibj.SPI; // SPI port, NavX gyro
import edu.wpi.first.wpilibj.XboxController; // controller

public class Robot extends TimedRobot {
  // selecting auto modes
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // motors
  private final CANSparkMax frontLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rearLeftMotor = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rearRightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax hookMotor = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax barMotor = new CANSparkMax(5, MotorType.kBrushless);
  SparkMaxPIDController barPID;
  // joysticks
  private final XboxController joystick = new XboxController(0);
  // gyro/navx
  private AHRS navx;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    hookMotor.setIdleMode(IdleMode.kBrake);
    try { navx = new AHRS(SPI.Port.kMXP); }
    catch (RuntimeException ex) 
    { 
      System.out.println("Failed to initialize NAVX "); 
    }
  }

  @Override
  public void robotPeriodic() 
  {
    //SmartDashboard.putNumber("NavX Pitch:", navx.getPitch());
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  // auto code
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  @Override
  public void teleopInit() {
    // setup barMotor PID
    barMotor.getEncoder().setPosition(0);
    barPID = barMotor.getPIDController();
    barPID.setP(0.034173);
    barPID.setI(0.0001);
    barPID.setOutputRange(-1, 0.1);
    hookMotor.setIdleMode(IdleMode.kBrake);
    SmartDashboard.putNumber("arm setpoint", -19.8);
  }

  @Override
  public void teleopPeriodic() 
  {
    if (joystick.getYButtonPressed()) { navx.zeroYaw(); }
    // driving
    double driveX = Math.pow(joystick.getRawAxis(0), 3);
    double driveY = Math.pow(joystick.getRawAxis(1), 3);
    double leftMotors = driveY - driveX;
    double rightMotors = driveY + driveX;
    frontLeftMotor.set(leftMotors);
    rearLeftMotor.set(-leftMotors);
    frontRightMotor.set(-rightMotors);
    rearRightMotor.set(rightMotors);
    // climbing
    // 0lx, 1ry, 4rx, 5ry joystick axis
    hookMotor.set(Math.pow(joystick.getRawAxis(5), 3));
    double armSetp = SmartDashboard.getNumber("arm setpoint", -6);
    // activate barMotor
    if (joystick.getXButton()) {
      barPID.setReference(armSetp, ControlType.kPosition);
      System.out.println("SETTING");
    }
    else {
      barMotor.set(0);
    }
    // barMotor.fsset(Math.pow(joystick.getRawAxis(4), 3));
    SmartDashboard.putNumber("bar enc", barMotor.getEncoder().getPosition());
  }

  @Override
  public void disabledInit() {
    hookMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("bar enc", barMotor.getEncoder().getPosition());}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}