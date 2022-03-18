// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;

//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

import frc.robot.commands.AutoAim;
import frc.robot.commands.FeedBall;
import frc.robot.commands.IntakeBall;

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
  // TODO: Change to Xbox and add in Operator control of intake and shooter
  public final Joystick joystick = new Joystick(Constants.Logitech.JOYSTICK_PORT);
  public final JoystickButton intakeButton = new JoystickButton(joystick, Constants.Logitech.BUTTON_1);
  public final JoystickButton feedButton = new JoystickButton(joystick, Constants.Logitech.BUTTON_2);
  public final JoystickButton autoAimButton = new JoystickButton(joystick, Constants.Logitech.BUTTON_4);
  // public final JoystickButton climbButton = new JoystickButton(joystick,
  // Constants.X_BUTTON);
  // public final JoystickButton traverseButton = new JoystickButton(joystick,
  // Constants.A_BUTTON);

  // subsystems
  private final Drive m_driveSubsystem = new Drive();
  // public final Climber m_climbSubsystem = new Climber(joystick);
  private final Shooter m_shooterSubsystem = new Shooter();
  private final Intake m_intakeSubsystem = new Intake();

  // commands
  // commands go here when read

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.manualDrive(joystick.getRawAxis(Constants.Logitech.LEFT_STICK_X_AXIS), joystick.getRawAxis(Constants.Logitech.LEFT_STICK_Y_AXIS))));
    m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> m_intakeSubsystem.setIntakeLift(joystick.getRawAxis(Constants.Logitech.RIGHT_STICK_Y_AXIS))));
    m_shooterSubsystem.setDefaultCommand(new RunCommand(() -> m_shooterSubsystem.manualAim(joystick)));
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
    intakeButton.whenHeld(new IntakeBall(m_intakeSubsystem));
    feedButton.whenHeld(new FeedBall(m_intakeSubsystem));
    autoAimButton.toggleWhenPressed(new AutoAim(m_shooterSubsystem, joystick));
    // climbButton.whenPressed(new Traverse(m_climbSubsystem, driveJoystick));
    // traverseButton.whenPressed(new TraverseBack(m_climbSubsystem,
    // driveJoystick));
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
