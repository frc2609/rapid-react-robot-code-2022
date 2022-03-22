// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.utils.SimPID;

public class ArmStartClimb extends CommandBase {
  /** Creates a new ArmStartClimb. */
  private Climber climber;
  private SimPID pid;

  // home first?
  public ArmStartClimb(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.pid = new SimPID(0.01, 0.0001, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setManualClimb(false);
    pid.setDesiredValue(climber.getArmKinematics().getArmAngle() + 96); // 96 deg up
    pid.setDoneRange(0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double val = pid.calcPID(climber.getArmKinematics().getArmAngle());
    System.out.println(val);
    climber.setArmVolt(val);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriverStation.reportWarning("ArmStartClimb done", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.isDone();
  }
}
