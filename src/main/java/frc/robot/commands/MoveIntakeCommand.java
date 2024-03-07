// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveIntakeCommand extends Command {

  IntakeSubsystem intake;
  double delta;
  
  /** Creates a new MoveIntakeCommand. */
  public MoveIntakeCommand(double delta, IntakeSubsystem intake) {
    this.delta = delta;
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.movePositionMotor(delta);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.movePositionMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (delta > 0 && intake.getPosition() > Constants.IntakeConstants.kDeployedPos - 2) || (delta < 0 && intake.getPosition() < -5);
  }
}
