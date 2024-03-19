// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterSubsystem;

public class JustShootAuto extends Command {

  ShooterSubsystem shooter;
  DriveTrain drive;

  /** Creates a new JustShootAuto. */
  public JustShootAuto(ShooterSubsystem shooter, DriveTrain drive) {
    this.shooter = shooter;
    this.drive = drive;
    addRequirements(shooter, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new RunCommand(shooter::shoot, shooter)
      .until(shooter::atSpeed)
      .andThen(shooter::feed)
      .andThen(new WaitCommand(0.15))
      .andThen(new RunCommand(() -> {
        drive.drive(1, 0, 0, true);
      }, drive)
      .withTimeout(1))
    .schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, true);
    shooter.stopFeed();
    shooter.stopShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
