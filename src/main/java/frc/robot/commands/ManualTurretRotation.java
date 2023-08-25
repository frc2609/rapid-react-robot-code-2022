// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret;

public class ManualTurretRotation extends CommandBase {
  private final Turret turret;
  private final CommandXboxController controller;

  /** Creates a new ManualTurretRotation. */
  public ManualTurretRotation(Turret turret, CommandXboxController controller) {
    this.turret = turret;
    this.controller = controller;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double left = -controller.getLeftTriggerAxis();
    final double right = controller.getRightTriggerAxis();
    turret.rotateTurret(
      (-left > right) ? left : right
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
