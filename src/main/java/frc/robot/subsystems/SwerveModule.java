// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwervePod. */

  private TalonFX _driveMotor;
  private CANSparkMax _angleMotor;
  private AbsoluteEncoder _absAngleEncoder;
  private RelativeEncoder _integratedNeoEncoder; 
  private ModPosition _podPos;
  private SparkPIDController _anglePID;
  private SwerveModuleState _targetState;
  private SwerveModuleState _currentState;
  private TalonFXSimState _talonSimState; 
  private LinearSystemSim<N1, N1, N1> _driveSim; 
  public enum ModPosition {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
  }
  public enum Motor {
    ANGLE,
    DRIVE 
  }
  public enum BrakeMode {
    BRAKE,
    COAST
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
    // config.Slot0.kP = Constants.SwerveChassis.kPDrive;
    // config.Slot0.kI = Constants.SwerveChassis.kIDrive;
    // config.Slot0.kD = Constants.SwerveChassis.kDDrive;
    // config.Slot0.kV = Constants.SwerveChassis.kVDrive;

    config.Slot0.kA = 0.01;
    config.Slot0.kD = 0.015;
    config.Slot0.kG = 0.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kP = 1.8;
    config.Slot0.kS = 1.75;
    config.Slot0.kV = 0.68;

    config.Feedback.SensorToMechanismRatio = 6.75;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1; 
    config.Voltage.PeakForwardVoltage = Constants.SwerveChassis.kPeakForwardFF;
    config.Voltage.PeakReverseVoltage = Constants.SwerveChassis.kPeakReverseFF;
    
    config.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -800; 
    _driveMotor.getConfigurator().apply(config);
    _driveMotor.setInverted(outputInverted);
    if(RobotBase.isSimulation()){
      _talonSimState = _driveMotor.getSimState(); 
      _driveSim = new LinearSystemSim<>(
        LinearSystemId.identifyVelocitySystem(
          Constants.SwerveChassis.kVDrive,
          Constants.SwerveChassis.kADrive
          )); 
    }
  }

  private void configureNeo(int id, boolean outputInverted) {
    _angleMotor = new CANSparkMax(id, MotorType.kBrushless);
    _angleMotor.restoreFactoryDefaults();
    if (_angleMotor.getStickyFault(FaultID.kSensorFault) || _angleMotor.getStickyFault(FaultID.kSensorFault))
    {
      System.out.println("Sensor fault Detected: falling back to NEO integratedEncoder.");

    }
    else {
      _absAngleEncoder = _angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
      _absAngleEncoder.setPositionConversionFactor(360);
    }
    _angleMotor.clearFaults();
    // minimize can bus traffic for angle motors. 
    _angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
    _angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
    _angleMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);

    _anglePID = _angleMotor.getPIDController();

    _integratedNeoEncoder = _angleMotor.getEncoder(); 
    _integratedNeoEncoder.setPositionConversionFactor(Constants.SwerveChassis.kAngleConversionFactor); 
    _integratedNeoEncoder.setPosition(_absAngleEncoder.getPosition());
    if (_absAngleEncoder != null){
      _anglePID.setFeedbackDevice(_absAngleEncoder);
    }
    else {
      _anglePID.setFeedbackDevice(_integratedNeoEncoder); 
    }
    _anglePID.setP(0.025);
    _anglePID.setI(0);
    _anglePID.setD(0.0);
    _anglePID.setFF(0.0008);
    _anglePID.setOutputRange(-1, 1);
    _anglePID.setPositionPIDWrappingEnabled(true);
    _anglePID.setPositionPIDWrappingMinInput(-180.0);
    _anglePID.setPositionPIDWrappingMaxInput(180.0);
    _angleMotor.setSmartCurrentLimit(40); 
    _angleMotor.setIdleMode(IdleMode.kBrake);

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
    Rotation2d angle; 
    if (_absAngleEncoder != null){
      angle = Rotation2d.fromDegrees(_absAngleEncoder.getPosition());
    } else {
      angle = Rotation2d.fromDegrees(_integratedNeoEncoder.getPosition()); 
    }
    return new SwerveModuleState(velocity, angle);
  }
  public SwerveModuleState getTargetState() {
    return _targetState; 
  }
  public void setDesiredState(SwerveModuleState desiredState) {
    
    if (!RobotBase.isSimulation()){
      desiredState = SwerveModuleState.optimize(desiredState, getState().angle);  
    }
    _targetState = desiredState;
    
    VelocityVoltage vv = new VelocityVoltage(
      0.0,
      0.0,
      false,
      0,
      0,
      false,
      false,
      false);
    vv.withVelocity(desiredState.speedMetersPerSecond / (2 * Math.PI * Constants.SwerveChassis.kWheelRadius));
    //if (Math.abs(vv.Velocity) == 0.0 )
      _driveMotor.setControl(vv);
    //else 
    //  _driveMotor.stopMotor();
    
    _anglePID.setReference(desiredState.angle.getDegrees(), CANSparkBase.ControlType.kPosition);

    //log();
  }
  public void setBrakeMode(Motor motor, BrakeMode mode)
  {
    if (motor == Motor.ANGLE){
      if (mode == BrakeMode.BRAKE) {
        _angleMotor.setIdleMode(IdleMode.kBrake); 
      } else {
        _angleMotor.setIdleMode(IdleMode.kCoast); 
      }
    } else {
      if (mode == BrakeMode.BRAKE) {
        _driveMotor.setNeutralMode(NeutralModeValue.Brake); 
      } else {
        _driveMotor.setNeutralMode(NeutralModeValue.Coast);; 
      }
    }
  }
  public void log() {
    var talonSignal = _driveMotor.getVelocity();

    switch (Constants.logLevel) {

      case SMART_DASH:
        SmartDashboard.putNumber(_podPos.name() + "/Target Angle", _targetState.angle.getDegrees());
        SmartDashboard.putNumber(_podPos.name() + "/Target Velocity (m per s)", _targetState.speedMetersPerSecond);
        SmartDashboard.putNumber(_podPos.name() + "/Current PID setpoint:",
            _targetState.angle.getDegrees());
        SmartDashboard.putNumber(_podPos.name() + "/Rotations Per Second", talonSignal.getValue());
        SmartDashboard.putNumber(_podPos.name() + "/Reported m per s",
            (talonSignal.getValue() / Constants.SwerveChassis.kDriveGearRatio) * 2 * Math.PI
                * Constants.SwerveChassis.kWheelRadius);
        SmartDashboard.putNumber(_podPos.name() + "/Current Module State/Angle", _currentState.angle.getDegrees());
        SmartDashboard.putNumber(_podPos.name() + "/Current Module State/Speed(m per S)", _currentState.speedMetersPerSecond);
        SmartDashboard.putNumber(_podPos.name() + "/Angle Motor Output", _angleMotor.get());
        if (_absAngleEncoder != null){
          SmartDashboard.putNumber(_podPos.name() + "/Absolute Angle Position", _absAngleEncoder.getPosition());
        }

        break;
      case PRINT:
        break;
      case SMART_DASH_AND_PRINT:
        break;
      default:
    }
  }

  public void testAngleMotor(double speed){
    _angleMotor.set(speed);
  }
  public void testAnglePosition(double setpoint)
  {
    _targetState.angle = Rotation2d.fromDegrees(setpoint);
    _anglePID.setReference(setpoint, ControlType.kPosition);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    var talonSignal = _driveMotor.getVelocity();
    if (_angleMotor.getStickyFault(FaultID.kSensorFault) || _angleMotor.getFault(FaultID.kSensorFault) && _absAngleEncoder != null)
    {
      System.out.println("Sensor fault Detected: falling back to NEO integratedEncoder.");
      _absAngleEncoder = null; 
      _anglePID.setFeedbackDevice(_integratedNeoEncoder); 
    }
    if (_absAngleEncoder != null){
      _currentState.angle = Rotation2d.fromDegrees(_absAngleEncoder.getPosition());
    }
    else {
      _currentState.angle = Rotation2d.fromDegrees(_integratedNeoEncoder.getPosition()); 
    }
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
  public SwerveModulePosition getEstimatedPosition()
  {
    var talonSignalEnc = _driveMotor.getPosition();
    double wheelRev = talonSignalEnc.getValue() / Constants.SwerveChassis.kDriveGearRatio;
    SwerveModulePosition pos = new SwerveModulePosition(wheelRev * Constants.SwerveChassis.metersPerRev,
        _targetState.angle);
    return pos;
  }
  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(_angleMotor, DCMotor.getNEO(1));
  }
  @Override
  public void simulationPeriodic(){
    _talonSimState.setSupplyVoltage(RobotController.getBatteryVoltage()); 
    _driveSim.setInput(_talonSimState.getMotorVoltage());
    // update input voltage 
    _driveSim.update(0.02);
    // update simulated talons 
    double driveVel = _driveSim.getOutput(0); 
    double motorRPS = meterToRotations(driveVel); 
    double motorRotations = motorRPS * 0.02; // Multiply by the loop rate 
    _talonSimState.addRotorPosition(motorRotations); 
    _talonSimState.setRotorVelocity(motorRPS); 
  }
  private double meterToRotations(double meters)
  {
    final double circumfrence = Constants.SwerveChassis.kWheelRadius * 2 * Math.PI; 
    final double wheelRotationsPerMeter = 1.0 / circumfrence; 
    return wheelRotationsPerMeter * meters; 
  }
  private double rotationToMeters(double rotations) {
     /* Get circumference of wheel */
     final double circumference = Constants.SwerveChassis.kWheelRadius * 2 * Math.PI;
     return rotations * circumference;
  }
}
