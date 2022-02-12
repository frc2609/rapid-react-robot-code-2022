// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import java.text.MessageFormat;
//import java.util.StringJoiner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetArm extends CommandBase {
  /** Creates a new SetArm. */
  private final Climber m_Climber;
  private final double desiredPosition;
  public SetArm(Climber climber, double angle) {
    m_Climber = climber;
    //addRequirements(climber); // TODO: Figure out how this works
    desiredPosition = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print(String.format("Setting arm to %.2f degrees", desiredPosition));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climber.setArmPosition(desiredPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.print(String.format("Arm is at %.2f degrees", desiredPosition));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return m_Climber.getPosition() == desiredPosition;
    return false;
  }
}
