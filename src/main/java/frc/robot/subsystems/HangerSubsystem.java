// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase {

  private CANSparkMax hangarMotor;
  private RelativeEncoder hangarEncoder;

  /** Creates a new HangarSubsystem. */
  public HangerSubsystem() {
    hangarMotor = new CANSparkMax(Constants.Mechanism.kHangerMotorId, MotorType.kBrushless);
    hangarEncoder = hangarMotor.getEncoder();
    hangarEncoder.setPosition(0);
  }

  public void extend() {
    if(hangarEncoder.getPosition() < -330) return;
    hangarMotor.set(-1.0);
    updateDashboardValues();
  }

  public void retract() {
    if(hangarEncoder.getPosition() >= -10) return;
    hangarMotor.set(1.0);
    updateDashboardValues();
  }

  public void extendUnsafe() {
    hangarMotor.set(-1.0);
    updateDashboardValues();
  }

  public void retractUnsafe() {
    hangarMotor.set(1.0);
    updateDashboardValues();
  }

  public void stop() {
    hangarMotor.set(0);
    updateDashboardValues();
  }

  private void updateDashboardValues() {
    SmartDashboard.putNumber("Hanger/height", hangarEncoder.getPosition());
  }

  @Override
  public void periodic() {}
}
