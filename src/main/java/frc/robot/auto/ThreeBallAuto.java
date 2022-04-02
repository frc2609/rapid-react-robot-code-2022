// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.MP.RamseteFactory;
import frc.robot.commands.autoaim.AutoAim;
import frc.robot.commands.autoaim.DisableFlywheel;
import frc.robot.commands.autoaim.TimedFeed;
import frc.robot.commands.autoaim.TrimShooter;
import frc.robot.commands.autonomous.AutoShoot;
import frc.robot.commands.autonomous.DriveAndExtendIntake;
import frc.robot.commands.autonomous.DriveStopCommand;
import frc.robot.commands.autonomous.PointTurn;
import frc.robot.commands.autonomous.ResetPose;
import frc.robot.commands.autonomous.ShootAndIntake;
import frc.robot.commands.autonomous.ShootAndIntakeHumanStation;
import frc.robot.commands.autonomous.TimerDelay;
import frc.robot.commands.autonomous.ZeroYaw;
import frc.robot.commands.intake.FeedBall;
import frc.robot.commands.intake.IntakeBall;
import frc.robot.commands.intake.SetBelt;
import frc.robot.commands.intake.StageBall;
import frc.robot.commands.intake.StopIntakeAndBelt;
import frc.robot.commands.intake.TimedIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new ThreeBallAuto. */
  public ThreeBallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    RamseteFactory factory = RamseteFactory.getInstance();
    System.out.println("*********************STARTING AUTONOMOUS");
    // addCommands(new
    // DriveAndExtendIntake(factory.constructRamseteCommand("startToBall")), new
    // DriveStopCommand(), new AutoaimShoot3s());
    addCommands(
        new DriveAndExtendIntake(factory.constructRamseteCommand("startToBall")),
        new DriveStopCommand(),
        new TimerDelay(0.2),
        new ShootAndIntake(),
        new AutoShoot(),
        new DisableFlywheel(),
        new StopIntakeAndBelt(),
        new PointTurn(120),
        new DriveStopCommand(),
        new ZeroYaw(),
        new ResetPose(factory.getTrajectory("secondToThird").getInitialPose()),
        new DriveAndExtendIntake(factory.constructRamseteCommand("secondToThird")),
        new DriveStopCommand(),
        new TimerDelay(0.2),
        new ShootAndIntake(),
        new AutoShoot(),
        new DisableFlywheel(),
        new StopIntakeAndBelt(),

        // below is 4 ball stuff (experimental)
        new ResetPose(factory.getTrajectory("thirdBallToForth").getInitialPose()),
        new DriveAndExtendIntake(factory.constructRamseteCommand("thirdBallToForth")),
        new DriveStopCommand(),
        new TimerDelay(0.2),
        // new ShootAndIntake(),
        new ShootAndIntakeHumanStation(),
        new AutoShoot()

    // factory.constructRamseteCommand("firstBallToSecondSetup"),
    // new DriveStopCommand(),
    // new ResetPose(factory.getTrajectory("secondSetupToBall").getInitialPose()),
    // new TimerDelay(0.2),
    // new
    // DriveAndExtendIntake(factory.constructRamseteCommand("secondSetupToBall")),
    //
    // new TrimShooter(200),
    // new IntakeBall(),
    // new AutoAim(),
    // new FeedBall(),
    // new TimerDelay(0.2),
    // new ZeroYaw(),
    // new ResetPose(factory.getTrajectory("crossTaxi").getInitialPose()),
    // factory.constructRamseteCommand("crossTaxi"),
    // new DriveStopCommand(),
    // new TrimShooter(0));
    );

    System.out.println("*********************ENDING AUTONOMOUS");
  }
}
