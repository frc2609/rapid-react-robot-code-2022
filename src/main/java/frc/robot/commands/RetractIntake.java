// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RetractIntake extends SequentialCommandGroup {
  /** Creates a new RetractIntake. */
  public RetractIntake(Intake intake) {
    addCommands(
        new InstantCommand(intake::retractIntake, intake),
        new InstantCommand(intake::intake, intake),
        Commands.waitSeconds(Constants.Intake.LIFT_TIME),
        new InstantCommand(intake::stopMotors, intake));
  }
}
