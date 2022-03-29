// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractIntakeStopBelt extends ParallelDeadlineGroup {
  /** Creates a new RetractIntake. */
  public RetractIntakeStopBelt() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new RetractIntakeTimer(1));
    addCommands(new SetIntakeBelt(0), new SetBelt(0), new ReverseUpperBeltTimer(0.2));
  }
}
