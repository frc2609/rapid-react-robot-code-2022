// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.EnableFlywheel;
import frc.robot.commands.ExtendIntake;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ManualDrive;
// import frc.robot.commands.ManualIntakeLift;
import frc.robot.commands.ManualTurretRotation;
import frc.robot.commands.OuttakeBall;
import frc.robot.commands.RetractIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Underglow;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // joystick
  private final CommandXboxController controller =
      new CommandXboxController(Constants.Xbox.CONTROLLER_PORT);
  // subsystems
  public final Drive driveSubsystem;
  public final Intake intakeSubsystem;
  public final Shooter shooterSubsystem;
  public final Turret turretSubsystem;
  public final Underglow underglowSubsystem;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveSubsystem = new Drive();
    intakeSubsystem =
        new Intake(
            Constants.Intake.INTAKE_SPEED,
            Constants.Intake.INTAKE_LIFT_SPEED,
            Constants.Intake.BELT_SPEED);
    shooterSubsystem = new Shooter();
    turretSubsystem = new Turret();
    underglowSubsystem = new Underglow();

    driveSubsystem.setDefaultCommand(new ManualDrive(driveSubsystem, controller));
    // intakeSubsystem.setDefaultCommand(new ManualIntakeLift(intakeSubsystem, controller));
    turretSubsystem.setDefaultCommand(new ManualTurretRotation(turretSubsystem, controller));
    underglowSubsystem.setDefaultCommand(
        new InstantCommand(underglowSubsystem::setPurple, underglowSubsystem));

    configureButtonBindings();
  }

  // define button mappings here
  private void configureButtonBindings() {
    // intake ball and store
    controller
        .a()
        .onTrue(
            new SequentialCommandGroup(
                new ExtendIntake(intakeSubsystem), new IntakeBall(intakeSubsystem)));
    controller.a().onFalse(new RetractIntake(intakeSubsystem));
    // outtake ball from storage
    controller
        .x()
        .onTrue(
            new SequentialCommandGroup(
                new ExtendIntake(intakeSubsystem), new OuttakeBall(intakeSubsystem)));
    // intake control
    controller.leftBumper().onTrue(new InstantCommand(intakeSubsystem::retractIntake));
    controller.rightBumper().onTrue(new InstantCommand(intakeSubsystem::extendIntake));
    // upper and lower belts
    controller.b().whileTrue(new InstantCommand(intakeSubsystem::allBeltsForward));
    controller.y().whileTrue(new InstantCommand(intakeSubsystem::allBeltsReverse));
    // flywheel
    controller.start().toggleOnTrue(new EnableFlywheel(shooterSubsystem));
    controller
        .povUp()
        .onTrue(new InstantCommand(shooterSubsystem::increaseFlywheelRPM, shooterSubsystem));
    controller
        .povDown()
        .onTrue(new InstantCommand(shooterSubsystem::decreaseFlywheelRPM, shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
