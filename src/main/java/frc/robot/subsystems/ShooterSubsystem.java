// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax lowMotor;
  private CANSparkMax highMotor;

  public ShooterSubsystem() {
    lowMotor = new CANSparkMax(Constants.Mechanism.kShooterLowMotorId, MotorType.kBrushless);
    highMotor = new CANSparkMax(Constants.Mechanism.kShooterHighMotorId, MotorType.kBrushless);
  }

  public void output(double speed) {
    lowMotor.set(speed);
    highMotor.set(speed);
  }

  @Override
  public void periodic() {}
}
