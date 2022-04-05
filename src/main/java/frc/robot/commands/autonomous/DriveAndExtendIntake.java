// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.autoaim.AutoAim;
import frc.robot.commands.intake.ExtendIntake;
// import frc.robot.commands.intake.IntakeBall;
import frc.robot.commands.intake.IntakeNoStage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndExtendIntake extends ParallelDeadlineGroup {
  /** Creates a new DriveAndExtendIntake. */
  public DriveAndExtendIntake(Command driveCommand) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(driveCommand, new ExtendIntake(),new AutoAim(), new IntakeNoStage());

    System.out.println("initializing DriveAndExtendIntake");
  }
}
