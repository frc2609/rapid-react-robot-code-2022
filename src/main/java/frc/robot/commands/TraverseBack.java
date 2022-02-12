// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class TraverseBack extends CommandBase {
  /** Creates a new Traverse. */
  private Climber m_climber;
  private Joystick m_stick;
  private double startPos = 0;
  // hook range 124.6
  public TraverseBack(Climber climber, Joystick stick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_stick = stick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = m_climber.getArmPosition();
    System.out.println("Arm start pos "+startPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setHook(-m_stick.getRawAxis(5));
    // start pos 98.15
    // end pos 77.64
    double progress = (Math.abs(m_climber.getHookPosition()-123))/(123);
    // System.out.println("Progress: " + progress);
    
    // System.out.println("Arm setp: " + (77.64+((101-77.64)*progress)));
    m_climber.setArmPosition(80+((101-80)*progress));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.resetHook();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_stick.getRawButton(2);
  }
}
