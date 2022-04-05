// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoaim;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/* Use with AUTONOMOUS MODE!
* Enables autoaim and the flywheel.
* Ends when autoaim aims at the target and doesn't disable the flywheel.
* The flywheel stays running so that auto can shoot the ball (FeedBall), and
* must be manually disabled with DisableFlywheel.
*/
public class AutoAim extends CommandBase {
  /** Creates a new AutoAim. */
  private Shooter m_shooter;
  double time,startTime;

  public AutoAim() {
    m_shooter = RobotContainer.m_shooterSubsystem;
    this.time = Constants.AutoConstants.commandTimer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.enableAutoAim();
    m_shooter.enableFlywheel();
    startTime = Timer.getFPGATimestamp();
    System.out.println("initializing AutoAim");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ending AutoAim");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // end the command when the target is locked so auto can continue sequence
    return RobotContainer.m_shooterSubsystem.isTargetLocked() || (Timer.getFPGATimestamp()>=(startTime+time));
  }
}
