// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbing;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.utils.SimPID;

public class ClimbAndReach extends CommandBase {
  /** Creates a new ArmStartClimb. */
  private Climber climber;
  private SimPID armPID;
  private final double hookPos = 110;
  // hook pos range = 110

  // home first?
  public ClimbAndReach(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.armPID = new SimPID(0.01, 0.0005, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setManualClimb(false);
    climber.setHookPosition(hookPos);
    armPID.setDesiredValue(climber.getArmKinematics().getArmAngle() - 15); // 96-81
    // deg up
    armPID.setDoneRange(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setHookPosition(hookPos);
    if (climber.getHookPosition() > 30) {
      double val = armPID.calcPID(climber.getArmKinematics().getArmAngle());
      climber.setArmVolt(val);
    }
    System.out.println(climber.getHookPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.disableHook();
    DriverStation.reportWarning("ClimbAndReach done", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
