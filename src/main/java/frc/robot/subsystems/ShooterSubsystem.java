// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax lowMotor;
  private CANSparkMax highMotor;
  private CANSparkMax feederMotor;

  private SparkPIDController lowMotorPID;
  private SparkPIDController highMotorPID;

  private RelativeEncoder lowMotorEncoder;
  private RelativeEncoder highMotorEncoder;

  public ShooterSubsystem() {
    lowMotor = new CANSparkMax(Constants.Mechanism.kShooterLowMotorId, MotorType.kBrushless);
    highMotor = new CANSparkMax(Constants.Mechanism.kShooterHighMotorId, MotorType.kBrushless);
    feederMotor = new CANSparkMax(Constants.Mechanism.kShooterFeederMotorId, MotorType.kBrushless);

    lowMotorEncoder = lowMotor.getEncoder();
    highMotorEncoder = highMotor.getEncoder();

    lowMotorPID = lowMotor.getPIDController();
    highMotorPID = highMotor.getPIDController();

    lowMotorPID.setP(0.0);
    lowMotorPID.setI(0.0);
    lowMotorPID.setD(0.0);
    lowMotorPID.setFF(0.0008);
    highMotorPID.setP(0.0);
    highMotorPID.setI(0.0);
    highMotorPID.setD(0.0);
    highMotorPID.setFF(0.0008);
  }

  public void output(double speed) {
    lowMotorPID.setReference(speed, ControlType.kVelocity);
    highMotorPID.setReference(speed, ControlType.kVelocity);
  }

  public void feed(double speed) {
    feederMotor.set(-speed);
  }

  @Override
  public void periodic() {
    log();
  }

  private void log() {
    SmartDashboard.putNumber("Shooter/Low Motor Speed", lowMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/High Motor Speed", highMotorEncoder.getVelocity());
  }
}
