// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.Traverse;
import frc.robot.commands.TraverseBack;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.XboxConstants;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ColorSensing;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // joysticks and buttons
  public final Joystick joystick = new Joystick(XboxConstants.JOYSTICK_PORT);
  //public final JoystickButton climbButton = new JoystickButton(joystick, Constants.X_BUTTON);
  //public final JoystickButton traverseButton = new JoystickButton(joystick, Constants.A_BUTTON);
  //public final JoystickButton ejectBallButton = new JoystickButton(driveJoystick, );

  // subsystems
  public final Drive m_driveSubsystem = new Drive(joystick);
  public final Climber m_climbSubsystem = new Climber(joystick);
  //public final ColorSensing m_colorSubsystem = new ColorSensing();
  public final Shooter m_shooterSubsystem = new Shooter(joystick);
  public final Intake m_intakeSubsystem = new Intake(joystick);

  // commands
  // commands go here when read

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // climbButton.whenPressed(new Traverse(m_climbSubsystem, driveJoystick));
    // traverseButton.whenPressed(new TraverseBack(m_climbSubsystem, driveJoystick));
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
