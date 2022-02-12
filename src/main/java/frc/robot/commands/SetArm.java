// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetArm extends CommandBase {
  /** Creates a new SetArm. */
  private final Climber m_climber;
  private final double m_desiredPosition;

  public SetArm(Climber climber, double angle) {
    m_climber = climber;
    //addRequirements(climber); // TODO: Figure out how this works
    m_desiredPosition = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("got here in set arm init");
    //System.out.println(String.format("Setting arm to %.2f degrees", desiredPosition));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("got here in set arm exec");
    System.out.println("position " + m_desiredPosition);
    m_climber.setArmPosition(m_desiredPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ended setarm at "+m_desiredPosition+" degrees");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.isArmInPosition(m_desiredPosition);
    //return false;
  }
}
