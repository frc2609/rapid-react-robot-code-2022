// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoaim;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.LockDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAimAndLock extends ParallelCommandGroup {
  /** Creates a new AutoAimAndLock. */
  public AutoAimAndLock() {
    // this will be used for teleop right? if not, change TeleopAutoAim for AutoAim
    addCommands(new TeleopAutoAim(), new LockDrive());
  }
}
