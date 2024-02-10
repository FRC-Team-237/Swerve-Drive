// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
/**
 * Represents the IntakeSubsystem class.
 * This class is responsible for controlling the intake subsystem of the robot.
 * It handles the deployment and retraction of the intake mechanism, as well as the intake and ejection of game objects.
 * The class uses CANSparkMax and PWMVictorSPX motor controllers for controlling the motors.
 * It also utilizes a SparkPIDController for position control and a RelativeEncoder for feedback.
 * The class provides methods for setting the intake motor speed, setting the position of the intake mechanism,
 * stopping the position motor, and getting the action command for a specific state.
 * It also includes a method for checking if the intake mechanism is in the fire position.
 * The class extends the SubsystemBase class and overrides the periodic method for periodic updates.
 */
public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_deployMotor; 
  private SparkPIDController m_deployController; 
  private RelativeEncoder m_deployEncoder; 
  private PWMVictorSPX m_intakeMotor; 
  private double m_intakePower = 0.0; 
  public enum Action {
    INTAKE, 
    EJECT, 
    LOAD, 
    FIRE
  }; 
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_deployMotor = new CANSparkMax(Constants.IntakeConstants.kDeployMotorId, MotorType.kBrushless); 
    m_deployController = m_deployMotor.getPIDController(); 
    m_deployEncoder = m_deployMotor.getEncoder(); 
    m_deployController.setFeedbackDevice(m_deployEncoder); 
    m_deployMotor.setIdleMode(IdleMode.kBrake); 
    m_intakeMotor = new PWMVictorSPX(Constants.IntakeConstants.kIntakeMotorId); 
    SmartDashboard.putNumber("Intake/P", 0.0);
    SmartDashboard.putNumber("Intake/I", 0.0);
    SmartDashboard.putNumber("Intake/D", 0.0);
    SmartDashboard.putNumber("Intake/FF", 0.00018125);   
    SmartDashboard.putNumber("Intake/Power", 0.5); 
    updateDashboardValues(); 
  }
  
  private void updateDashboardValues() {
    double P = SmartDashboard.getNumber("Intake/P", 0.0);
    double I = SmartDashboard.getNumber("Intake/I", 0.0);
    double D = SmartDashboard.getNumber("Intake/D", 0.0);
    double FF = SmartDashboard.getNumber("Intake/FF", 0.00018125);

    m_deployController.setP(P);
    m_deployController.setI(I); 
    m_deployController.setD(D); 
    m_deployController.setFF(FF); 
    m_intakePower = SmartDashboard.getNumber("Intake/Power", 0.5);
  }
  public void setIntakeMotor(double speed){
    m_intakeMotor.set(speed);
  }
  public void setPosition(double pos){
    m_deployController.setReference(pos, ControlType.kPosition);
  }
  public void stopPositionMotor(){
    m_deployMotor.stopMotor();
  }
/**
 * Returns the command associated with the given action state.
 * 
 * @param state the action state
 * @return the command associated with the action state
 */  public Command getActionCommand(Action state){
    switch (state){
      case INTAKE:
      return new InstantCommand(() -> {
        this.setPosition(Constants.IntakeConstants.kDeployedPos);
        this.setIntakeMotor(0.0);
      }, (Subsystem) this); 
      case LOAD:
      return new InstantCommand(() -> {
        this.setPosition(Constants.IntakeConstants.kRetractedPos);
        this.setIntakeMotor(m_intakePower);
      }, (Subsystem) this); 
      case EJECT:
      return new InstantCommand(() -> this.setPosition(Constants.IntakeConstants.kDeployedPos), (Subsystem)this)
      .andThen(new WaitCommand(0.5))
      .andThen(() -> this.setIntakeMotor(-m_intakePower),(Subsystem)this); 
      case FIRE:
      return new WaitUntilCommand(this::inFirePosition).andThen(()-> setIntakeMotor(-m_intakePower), (Subsystem) this); 
      default:
      return new InstantCommand(); 
    }
  }
  public boolean inFirePosition()
  {
    return Math.abs(m_deployEncoder.getPosition() - Constants.IntakeConstants.kRetractedPos) < 5; 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
