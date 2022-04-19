// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.MP.RamseteFactory;
import frc.robot.commands.autoaim.DisableAutoAim;
import frc.robot.commands.autonomous.AutoShoot;
import frc.robot.commands.autonomous.DriveAndExtendIntake;
import frc.robot.commands.autonomous.DriveCommand;
import frc.robot.commands.autonomous.DriveStopCommand;
import frc.robot.commands.autonomous.PointTurn;
import frc.robot.commands.autonomous.ResetPose;
import frc.robot.commands.autonomous.Spit;
import frc.robot.commands.autonomous.TimerDelay;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SecondPickTwoBallAuto extends SequentialCommandGroup {
  /** Creates a new SecondPickTwoBallAuto. */
  RamseteFactory factory = RamseteFactory.getInstance();

  public SecondPickTwoBallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      factory.constructRamseteCommand("SecondPickDriveForwardToShoot"),
      new DriveStopCommand(),
      new TimerDelay(0.2),
      new AutoShoot()
      // new ResetPose(factory.getTrajectory("SecondPickDriveForwardToWrongBall").getInitialPose()),
      // new DriveAndExtendIntake(factory.constructRamseteCommand("SecondPickDriveForwardToWrongBall")),
      // new DriveStopCommand(),
      // new TimerDelay(0.2),
      // new DisableAutoAim(),
      // new PointTurn(-30),
      // new DriveStopCommand(),
      // new Spit()
    );
  }
}
