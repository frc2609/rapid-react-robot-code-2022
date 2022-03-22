// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MP.RamseteFactory;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoaimShoot3s;
import frc.robot.commands.DisableAutoAim;
import frc.robot.commands.DriveAndExtendIntake;
import frc.robot.commands.DriveStopCommand;
import frc.robot.commands.FeedBall;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.Print;
import frc.robot.commands.ResetPose;
import frc.robot.commands.StageBall;
import frc.robot.commands.DisableFlywheel;
import frc.robot.commands.StopIntakeAndBelt;
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
    // addCommands(new DriveAndExtendIntake(factory.constructRamseteCommand("startToBall")), new DriveStopCommand(), new AutoaimShoot3s());
    addCommands(new DriveAndExtendIntake(factory.constructRamseteCommand("startToBall")), new DriveStopCommand(), new ResetPose(factory.getTrajectory("firstBallToSecondSetup").getInitialPose()),
     new TimerDelay(0.2),
      new IntakeBall(),
      new AutoAim(),
      new FeedBall(),new AutoAim(),new StageBall(),  new FeedBall(),  new TimerDelay(0.5), new DisableFlywheel(), new StopIntakeAndBelt(),
      factory.constructRamseteCommand("firstBallToSecondSetup"),new DriveStopCommand(),new ResetPose(factory.getTrajectory("secondSetupToBall").getInitialPose()),
      new TimerDelay(0.2),
      new DriveAndExtendIntake(factory.constructRamseteCommand("secondSetupToBall")), new DriveStopCommand(), new IntakeBall(), new AutoAim(), new FeedBall());
  }
}
