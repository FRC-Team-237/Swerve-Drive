// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwervePod extends SubsystemBase {
  /** Creates a new SwervePod. */
  
  private TalonFX _driveMotor; 
  private CANSparkMax _angleMotor; 
  private RelativeEncoder _angleEncoder; 
  private PodPosition _podPos; 
  private Rotation2d _currentAngle = new Rotation2d();
  public enum PodPosition {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }
  SimpleMotorFeedforward _ff = new SimpleMotorFeedforward(
    Constants.SwerveChassis.kSDrive,
    Constants.SwerveChassis.kVDrive, 
    Constants.SwerveChassis.kADrive
   );  
  public SwervePod(PodPosition pos ) {
    _podPos = pos; 
    configureDriveMotor();
    configureAngleMotor();
  }
  private void configureTalon(int id, boolean outputInverted, boolean sensorInverted)
  {
    _driveMotor = new TalonFX(id); 
    // default the configuration
    _driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    _driveMotor.setInverted(outputInverted);
     
  }
  private void configureNeo(int id, boolean outputInverted){
    _angleMotor = new CANSparkMax(id, MotorType.kBrushless); 
    _angleEncoder = _angleMotor.getEncoder(); 
  }
  private void configureDriveMotor()
  {
    switch (_podPos){
      case FRONT_LEFT:
      configureTalon(
        Constants.SwerveChassis.FrontLeft.kDriveId,
        Constants.SwerveChassis.FrontLeft.kDriveInverted,
        Constants.SwerveChassis.FrontLeft.kDriveSensorPhaseInverted);
      break; 
      case FRONT_RIGHT:
      configureTalon(
        Constants.SwerveChassis.FrontRight.kDriveId,
        Constants.SwerveChassis.FrontRight.kDriveInverted,
        Constants.SwerveChassis.FrontRight.kDriveSensorPhaseInverted);
      break; 
      case BACK_LEFT:
      configureTalon(
        Constants.SwerveChassis.BackLeft.kDriveId,
        Constants.SwerveChassis.BackLeft.kDriveInverted,
        Constants.SwerveChassis.BackLeft.kDriveSensorPhaseInverted);
      break; 
      case BACK_RIGHT:
      configureTalon(
        Constants.SwerveChassis.BackRight.kDriveId,
        Constants.SwerveChassis.BackRight.kDriveInverted,
        Constants.SwerveChassis.BackRight.kDriveSensorPhaseInverted);
      break; 
    }
  }
  private void configureAngleMotor()
  {
    switch (_podPos){
      case FRONT_LEFT:
      configureNeo(
        Constants.SwerveChassis.FrontLeft.kDriveId,
        Constants.SwerveChassis.FrontLeft.kDriveInverted);
      break; 
      case FRONT_RIGHT:
      configureNeo(
        Constants.SwerveChassis.FrontRight.kDriveId,
        Constants.SwerveChassis.FrontRight.kDriveInverted);
      break; 
      case BACK_LEFT:
      configureNeo(
        Constants.SwerveChassis.BackLeft.kDriveId,
        Constants.SwerveChassis.BackLeft.kDriveInverted);
      break; 
      case BACK_RIGHT:
      configureNeo(
        Constants.SwerveChassis.BackRight.kDriveId,
        Constants.SwerveChassis.BackRight.kDriveInverted);
      break; 
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
