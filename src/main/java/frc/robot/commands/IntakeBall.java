// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeBall extends CommandBase {
  private Intake m_intake;
  private int whereToIntake = -1; // 1 - stage, 2 - intake, 0 - intake needs staging

  public IntakeBall() {
    m_intake = RobotContainer.m_intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.m_shooterSubsystem.stagingSensor.get()){
      whereToIntake = 2; // staging sensor populated
    }else if(RobotContainer.m_shooterSubsystem.getIntakeSensor()){
      whereToIntake = 0; // staging sensor free, intake taken
    } else if(!RobotContainer.m_shooterSubsystem.stagingSensor.get() && !RobotContainer.m_shooterSubsystem.getIntakeSensor()){
      whereToIntake = 1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (whereToIntake == 0 && !RobotContainer.m_shooterSubsystem.stagingSensor.get()){
      m_intake.setIntake(0);
      m_intake.setBelts(Constants.Motors.BELT_SPEED*0.5);
    }else if(whereToIntake == 0 && RobotContainer.m_shooterSubsystem.stagingSensor.get()){
      m_intake.setBelts(0);
      whereToIntake = 2;
    }

    if(whereToIntake == 1 && !RobotContainer.m_shooterSubsystem.stagingSensor.get()){
      m_intake.setIntake(Constants.Motors.INTAKE_SPEED);
      m_intake.setBelts(Constants.Motors.BELT_SPEED*0.5);
    }else if(whereToIntake == 1 && RobotContainer.m_shooterSubsystem.stagingSensor.get()){
      m_intake.setBelts(0);
    }

    if(whereToIntake == 2 && !RobotContainer.m_shooterSubsystem.getIntakeSensor()){
      m_intake.setIntake(Constants.Motors.INTAKE_SPEED);
      m_intake.setLowerBelt(Constants.Motors.BELT_SPEED*0.5);
    }else if(whereToIntake == 2 && RobotContainer.m_shooterSubsystem.getIntakeSensor()){
      m_intake.setBelts(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntake(0.0);
    m_intake.setBelts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(whereToIntake == 1){
      return RobotContainer.m_shooterSubsystem.stagingSensor.get();
    }else if(whereToIntake == 2){
      return RobotContainer.m_shooterSubsystem.getIntakeSensor();
    }else{
      return false;
    }

  }
}
