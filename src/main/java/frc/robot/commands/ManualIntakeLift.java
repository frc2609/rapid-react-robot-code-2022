// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Xbox;
import frc.robot.subsystems.Intake;

public class ManualIntakeLift extends CommandBase {
  private final Intake intake;
  private final CommandXboxController controller;

  /** Creates a new ManualIntakeLift. */
  public ManualIntakeLift(Intake intake, CommandXboxController controller) {
    this.intake = intake;
    this.controller = controller;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeLift(MathUtil.applyDeadband(controller.getRightY(), Xbox.DEADBAND));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
