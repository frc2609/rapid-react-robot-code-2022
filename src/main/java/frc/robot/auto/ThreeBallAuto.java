// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MP.RamseteFactory;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoaimShoot2s;
import frc.robot.commands.DriveAndExtendIntake;
import frc.robot.commands.DriveStopCommand;
import frc.robot.commands.FeedBall;
import frc.robot.commands.TimerDelay;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new ThreeBallAuto. */
  public ThreeBallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    RamseteFactory factory = RamseteFactory.getInstance();
    addCommands(new DriveAndExtendIntake(factory.startToBallCommand), new DriveStopCommand(), new AutoaimShoot2s());
  }
}
