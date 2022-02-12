// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbingSequence extends SequentialCommandGroup {
  /** Creates a new ClimbingSequence. */
  public ClimbingSequence(Climber climber, Joystick stick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // start pos 98.15
    // end pos 77.64
    addCommands(new SetArm(climber, 98.15), new WaitForButtonPress(stick, 4));
  }
}
