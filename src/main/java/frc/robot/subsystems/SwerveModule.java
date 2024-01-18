// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwervePod. */

  private TalonFX _driveMotor;
  private CANSparkMax _angleMotor;
  private RelativeEncoder _angleEncoder;
  private ModPosition _podPos;
  private SparkPIDController _anglePID;
  private SwerveModuleState _targetState;
  private SwerveModuleState _currentState;

  public enum ModPosition {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }

  SimpleMotorFeedforward _ff = new SimpleMotorFeedforward(
      Constants.SwerveChassis.kSDrive,
      Constants.SwerveChassis.kVDrive,
      Constants.SwerveChassis.kADrive);

  public SwerveModule(ModPosition pos) {
    _podPos = pos;
    _targetState = new SwerveModuleState();
    _currentState = new SwerveModuleState();
    this.setSubsystem("Swerve Module");
    switch (pos) {
      case FRONT_LEFT:
        this.setName("Front Left Module");
        break;
      case FRONT_RIGHT:
        this.setName("Front Right Module");
        break;
      case BACK_LEFT:
        this.setName("Back Left Module");
        break;
      case BACK_RIGHT:
        this.setName("Back Right Module");
        break;
    }
    configureDriveMotor();
    configureAngleMotor();
  }

  private void configureTalon(int id, boolean outputInverted, boolean sensorInverted) {
    _driveMotor = new TalonFX(id);
    // set up config
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = Constants.SwerveChassis.kPDrive;
    config.Slot0.kI = Constants.SwerveChassis.kIDrive;
    config.Slot0.kD = Constants.SwerveChassis.kDDrive;
    config.Slot0.kV = Constants.SwerveChassis.kVDrive;
    config.Voltage.PeakForwardVoltage = Constants.SwerveChassis.kPeakForwardFF;
    config.Voltage.PeakReverseVoltage = Constants.SwerveChassis.kPeakReverseFF;

    _driveMotor.getConfigurator().apply(config);
    _driveMotor.setInverted(outputInverted);
  }

  private void configureNeo(int id, boolean outputInverted) {
    _angleMotor = new CANSparkMax(id, MotorType.kBrushless);
    _angleEncoder = _angleMotor.getEncoder();
    // _angleEncoder = _angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    _anglePID = _angleMotor.getPIDController();
    _anglePID.setFeedbackDevice(_angleEncoder);
    _anglePID.setP(0.4);
    _anglePID.setI(0);
    _anglePID.setD(0.8);
    _anglePID.setOutputRange(-1, 1);
    _anglePID.setPositionPIDWrappingEnabled(true);
    _anglePID.setPositionPIDWrappingMinInput(0);
    _anglePID.setPositionPIDWrappingMaxInput(Math.PI * 2.0);
    _angleMotor.restoreFactoryDefaults();
    _angleMotor.clearFaults();
    _angleMotor.setIdleMode(IdleMode.kBrake);
    _angleEncoder.setPosition(0.0);

    _angleMotor.burnFlash();
  }

  private void configureDriveMotor() {
    switch (_podPos) {
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

  private void configureAngleMotor() {
    switch (_podPos) {
      case FRONT_LEFT:
        configureNeo(
            Constants.SwerveChassis.FrontLeft.kAngleId,
            Constants.SwerveChassis.FrontLeft.kAngleInverted);
        break;
      case FRONT_RIGHT:
        configureNeo(
            Constants.SwerveChassis.FrontRight.kAngleId,
            Constants.SwerveChassis.FrontRight.kAngleInverted);
        break;
      case BACK_LEFT:
        configureNeo(
            Constants.SwerveChassis.BackLeft.kAngleId,
            Constants.SwerveChassis.BackLeft.kAngleInverted);
        break;
      case BACK_RIGHT:
        configureNeo(
            Constants.SwerveChassis.BackRight.kAngleId,
            Constants.SwerveChassis.BackRight.kAngleInverted);
        break;
    }
  }

  public SwerveModuleState getState() {
    double velocity = _driveMotor.getVelocity().getValue();
    Rotation2d angle = Rotation2d.fromDegrees(_angleEncoder.getPosition() * Constants.SwerveChassis.degreesPerTick);
    return new SwerveModuleState(velocity, angle);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    _targetState = desiredState;
    VelocityVoltage vv = new VelocityVoltage(
      0.0,
      0.0,
      true,
      0,
      0,
      false,
      false,
      false);
    vv.withVelocity(desiredState.speedMetersPerSecond / (2 * Math.PI * Constants.SwerveChassis.kWheelRadius)
        * Constants.SwerveChassis.kDriveGearRatio);
    _driveMotor.setControl(vv);
    
    double angleToEncPos = desiredState.angle.getDegrees() / Constants.SwerveChassis.degreesPerTick;
    SmartDashboard.putNumber(_podPos.name() + "/Angle to Encoder Position", angleToEncPos);
    _angleMotor.getPIDController().setReference(angleToEncPos, CANSparkBase.ControlType.kPosition);

    log();
  }

  public void log() {
    var talonSignal = _driveMotor.getVelocity();

    switch (Constants.logLevel) {

      case SMART_DASH:
        SmartDashboard.putNumber(_podPos.name() + "/Target Angle", _targetState.angle.getDegrees());
        SmartDashboard.putNumber(_podPos.name() + "/Target Velocity (m per s)", _targetState.speedMetersPerSecond);
        SmartDashboard.putNumber(_podPos.name() + "/Reported Angle",
            _angleEncoder.getPosition() * Constants.SwerveChassis.degreesPerTick);
        SmartDashboard.putNumber(_podPos.name() + "/Raw Angle Encoder",
            _angleEncoder.getPosition());
        SmartDashboard.putNumber(_podPos.name() + "/Current PID setpoint:",
            _targetState.angle.getDegrees() / Constants.SwerveChassis.degreesPerTick);
        SmartDashboard.putNumber(_podPos.name() + "/Rotations Per Second", talonSignal.getValue());
        SmartDashboard.putNumber(_podPos.name() + "/Reported m per s",
            (talonSignal.getValue() / Constants.SwerveChassis.kDriveGearRatio) * 2 * Math.PI
                * Constants.SwerveChassis.kWheelRadius);
        break;
      case PRINT:
        break;
      case SMART_DASH_AND_PRINT:
        break;
      default:
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var talonSignal = _driveMotor.getVelocity();
    _currentState.angle = new Rotation2d(_angleEncoder.getPosition() * Constants.SwerveChassis.degreesPerTick);
    double wheelRPS = talonSignal.getValue() / Constants.SwerveChassis.kDriveGearRatio;
    _currentState.speedMetersPerSecond = (wheelRPS) * 2 * Math.PI * Constants.SwerveChassis.kWheelRadius;
    log();
  }

  public SwerveModulePosition getPosition() {
    var talonSignalEnc = _driveMotor.getPosition();
    double wheelRev = talonSignalEnc.getValue() / Constants.SwerveChassis.kDriveGearRatio;
    SwerveModulePosition pos = new SwerveModulePosition(wheelRev * Constants.SwerveChassis.metersPerRev,
        _currentState.angle);
    return pos;
  }
}
