// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax shooterMotor;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotor = new CANSparkMax(Constants.Mechanism.kShooterMotorId, MotorType.kBrushless);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {


  }
}
