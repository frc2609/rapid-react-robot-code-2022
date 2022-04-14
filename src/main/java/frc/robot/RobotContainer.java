// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Underglow;
import frc.robot.MP.Looper;
// import frc.robot.MP.RamseteFactory;
import frc.robot.commands.autoaim.AutoAimAndLock;
// import frc.robot.commands.autonomous.PointTurn;
import frc.robot.commands.intake.ExtendIntakeRunBelt;
import frc.robot.commands.intake.IntakeReverse;
import frc.robot.commands.intake.RetractIntakeStopBelt;
import frc.robot.commands.intake.ReverseUpperBelt;
import frc.robot.commands.intake.ReverseUpperBeltTimer;
import frc.robot.commands.intake.TeleopFeedBall;
//import frc.robot.commands.intake.TeleopIntakeBall;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import frc.robot.auto.TwoBallAuto;
import frc.robot.auto.ThreeBallAuto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // joysticks and buttons
  public static Joystick driveJoystick = new Joystick(Constants.Xbox.DRIVER_PORT);
  public static Joystick operatorJoystick = new Joystick(Constants.Xbox.OPERATOR_PORT);
  public static JoystickButton intakeButton = new JoystickButton(driveJoystick, Constants.Xbox.A_BUTTON);
  public static JoystickButton feedButton = new JoystickButton(operatorJoystick, Constants.Xbox.B_BUTTON);
  public static JoystickButton autoAimButton = new JoystickButton(operatorJoystick, Constants.Xbox.A_BUTTON);
  public static JoystickButton outtakeButton = new JoystickButton(driveJoystick, Constants.Xbox.START_BUTTON);
  public static JoystickButton reverseUpperBeltButton = new JoystickButton(operatorJoystick, Constants.Xbox.X_BUTTON);
  public static JoystickButton testButton = new JoystickButton(driveJoystick, 7);
  // subsystems
  public static Drive m_driveSubsystem;
  public static Climber m_climbSubsystem;
  public static Shooter m_shooterSubsystem;
  public static Intake m_intakeSubsystem;
  public static Underglow m_underglowSubsystem;
  public static AHRS bodyNavx;

  public Looper enabledLooper;

  public CvSink m_cvSink;
  public CvSource m_outputStream;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      bodyNavx = new AHRS(SerialPort.Port.kMXP);
    } catch (RuntimeException e) {
      DriverStation.reportError("BodyNavx failed to initialize", false);
    }
    m_driveSubsystem = new Drive();
    m_climbSubsystem = new Climber();
    m_intakeSubsystem = new Intake();
    m_shooterSubsystem = new Shooter();
    m_underglowSubsystem = new Underglow();

    enabledLooper = new Looper();

    configureButtonBindings();
    // try {
    // enabledLooper.register(m_driveSubsystem.getLooper());
    // } catch (Throwable t) {
    // System.out.println(t.getMessage());
    // System.out.println(t.getStackTrace());
    // }
  }

  // define button mappings here
  private void configureButtonBindings() {
    // intakeButton.whenHeld(new TeleopIntakeBall());
    // intakeButton.whenReleased(new ReverseUpperBeltTimer(0.2));
    intakeButton.whenHeld(new ExtendIntakeRunBelt());
    intakeButton.whenReleased(new RetractIntakeStopBelt());
    autoAimButton.toggleWhenPressed(new AutoAimAndLock());
    feedButton.whileHeld(new TeleopFeedBall());
    feedButton.whenReleased(new ReverseUpperBeltTimer(0.2));
    outtakeButton.whileHeld(new IntakeReverse());
    reverseUpperBeltButton.whileHeld(new ReverseUpperBelt());
    // testButton.whenPressed(RamseteFactory.getInstance().constructRamseteCommand("secondToThird"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
    boolean isTwoBall = SmartDashboard.getBoolean("Enable 2 Ball Auto", false);
    if (isTwoBall) return new TwoBallAuto();
    else return new ThreeBallAuto();
  }
}
