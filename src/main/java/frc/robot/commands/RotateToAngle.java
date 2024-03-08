// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class RotateToAngle extends Command {

  DriveTrain drive;
  double angle;

  public RotateToAngle(double angle, DriveTrain drive) {
    this.angle = angle;
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // drive.setIsAutoRotating(true);
    drive.targetAngle = angle;
    System.out.println("Distance from " + drive.targetAngle + " to " + drive.getAngle() + " is " + drive.realDistance(drive.targetAngle));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.targetDelta = 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("Ended auto rotate");
    // drive.setIsAutoRotating(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.isNear(angle);
  }
}
