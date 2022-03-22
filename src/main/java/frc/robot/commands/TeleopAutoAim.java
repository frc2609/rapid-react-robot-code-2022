// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

/* USE THIS FOR TELEOP ONLY! DON'T USE IT FOR AUTO!
*  Command will never end by itself, so trigger it with "togglewhenpressed".
*  Enables autoaim when initialized, and disables it when ended.
*  By using "togglewhenpressed", you can press one button to toggle autoaim
*  instead of holding the button down to use autoaim.
*/
public class TeleopAutoAim extends CommandBase {
  /** Creates a new AutoAim. */
  private Shooter m_shooter;

  public TeleopAutoAim() {
    m_shooter = RobotContainer.m_shooterSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.enableAutoAim();
    m_shooter.enableFlywheel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.disableAutoAim();
    m_shooter.disableFlywheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
