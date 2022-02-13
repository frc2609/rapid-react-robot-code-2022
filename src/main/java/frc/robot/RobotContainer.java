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

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ColorSensing;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // joysticks and buttons
  public final Joystick driveJoystick = new Joystick(0);
  public final JoystickButton climbButton = new JoystickButton(driveJoystick, 3); // X on xbox controller
  public final JoystickButton traverseButton = new JoystickButton(driveJoystick, 1);
  //public final JoystickButton ejectBallButton = new JoystickButton(driveJoystick, );

  // subsystems
  private final Drive m_driveSubsystem = new Drive(driveJoystick);
  private final Climber m_climbSubsystem = new Climber(driveJoystick);
  private final ColorSensing m_colorSubsystem = new ColorSensing();

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
    climbButton.whenPressed(new Traverse(m_climbSubsystem, driveJoystick));
    traverseButton.whenPressed(new TraverseBack(m_climbSubsystem, driveJoystick));
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
