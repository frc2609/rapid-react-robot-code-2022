// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.ThreeBallAuto;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  Command x;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // printVersion();
    x = new ThreeBallAuto();

    SmartDashboard.putBoolean(Constants.INTAKE_OVERRIDE_STRING, false);
    SmartDashboard.putBoolean(Constants.FEEDER_OVERRIDE_STRING, false);
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("intakeSensor", RobotContainer.m_shooterSubsystem.getIntakeSensor());
    SmartDashboard.putBoolean("stagingSensor", RobotContainer.m_shooterSubsystem.stagingSensor.get());
    SmartDashboard.putBoolean("shooterSensor", RobotContainer.m_shooterSubsystem.shooterSensor.get());

    // RamseteFactory.getInstance().printPath();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // m_robotContainer.m_climbSubsystem.setArmToZero();
    m_robotContainer.enabledLooper.stop();
    RobotContainer.m_driveSubsystem.setBrake(false);
    RobotContainer.m_shooterSubsystem.disableAutoAim();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommand = x;
    RobotContainer.m_driveSubsystem.resetEncoders();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.m_driveSubsystem.setBrake(true);
    RobotContainer.bodyNavx.zeroYaw();
    m_robotContainer.enabledLooper.start();
    RobotContainer.m_driveSubsystem.resetOdometry(new Pose2d());
    RobotContainer.m_underglowSubsystem.checkColor();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    RobotContainer.m_driveSubsystem.setBrake(true);
    RobotContainer.m_driveSubsystem.resetOdometry(new Pose2d());
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.enabledLooper.start();
    RobotContainer.m_shooterSubsystem.disableAutoAim(); // auto will leave it running, disables at start of teleop
    RobotContainer.m_underglowSubsystem.checkColor();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    RobotContainer.m_driveSubsystem.manualDrive(RobotContainer.driveJoystick.getRawAxis(Constants.Xbox.LEFT_STICK_X_AXIS), RobotContainer.driveJoystick.getRawAxis(Constants.Xbox.LEFT_STICK_Y_AXIS));
    RobotContainer.m_intakeSubsystem.setIntakeLift(-RobotContainer.driveJoystick.getRawAxis(Constants.Xbox.RIGHT_STICK_Y_AXIS));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
