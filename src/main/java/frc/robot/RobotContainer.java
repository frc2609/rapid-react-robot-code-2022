// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climber;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.MP.Looper;
import frc.robot.MP.RamseteFactory;
import frc.robot.auto.ThreeBallAuto;
import frc.robot.commands.AutoAim;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.FeedBall;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.StageBall;
import frc.robot.commands.LockDrive;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;


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
  // subsystems
  public static Drive m_driveSubsystem;
  public static Climber m_climbSubsystem;
  public static Shooter m_shooterSubsystem;
  public static Intake m_intakeSubsystem;
  public static AHRS bodyNavx;
  
  public Looper enabledLooper;

  public CvSink m_cvSink;
  public CvSource m_outputStream;

  // commands
  // commands go here when read

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    try {
      bodyNavx = new AHRS(SerialPort.Port.kMXP);
    }catch(RuntimeException e){
      DriverStation.reportError("BodyNavx failed to initialize", false);
    }
    m_driveSubsystem = new Drive();
    m_climbSubsystem = new Climber();
    m_intakeSubsystem = new Intake();
    m_shooterSubsystem = new Shooter();

    // m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.manualDrive(driveJoystick.getRawAxis(Constants.Xbox.LEFT_STICK_X_AXIS), driveJoystick.getRawAxis(Constants.Xbox.LEFT_STICK_Y_AXIS)), m_driveSubsystem));
    // m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> m_intakeSubsystem.setIntakeLift(-driveJoystick.getRawAxis(Constants.Xbox.RIGHT_STICK_Y_AXIS)), m_intakeSubsystem));
    // m_shooterSubsystem.setDefaultCommand(new RunCommand(() -> m_shooterSubsystem.manualAim(operatorJoystick), m_shooterSubsystem));

    enabledLooper = new Looper();

    UsbCamera usbCamera = new UsbCamera("USB Camera", 0);
    MjpegServer mjpegServer1 = new MjpegServer("USB Camera Server", 1181);
    mjpegServer1.setSource(usbCamera);

    configureButtonBindings();
    // try {
    //   enabledLooper.register(m_driveSubsystem.getLooper());
    // } catch (Throwable t) {
    //   System.out.println(t.getMessage());
    //   System.out.println(t.getStackTrace());
    // }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // intakeButton.whenHeld(new IntakeBall());
    intakeButton.whenHeld(new LockDrive());

    feedButton.whenHeld(new FeedBall());
    // autoAimButton.toggleWhenPressed(new AutoAim(m_shooterSubsystem, operatorJoystick));
    autoAimButton.toggleWhenPressed(new StageBall());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Put autonomous command here when ready
    return null;
  }
}
