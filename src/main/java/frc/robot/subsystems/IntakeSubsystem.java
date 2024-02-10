// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax m_deployMotor; 
  SparkPIDController m_deployController; 
  RelativeEncoder m_deployEncoder; 
  PWMVictorSPX m_intakeMotor; 
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    
    m_deployMotor = new CANSparkMax(Constants.IntakeConstants.kDeployMotorId, MotorType.kBrushless); 
    m_deployController = m_deployMotor.getPIDController(); 
    m_deployEncoder = m_deployMotor.getEncoder(); 
    m_deployMotor.setIdleMode(IdleMode.kBrake); 
    m_intakeMotor = new PWMVictorSPX(Constants.IntakeConstants.kIntakeMotorId); 
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
