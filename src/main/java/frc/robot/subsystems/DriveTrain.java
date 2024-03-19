// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveChassis;
import frc.robot.Utilities.NavUtilites;
import frc.robot.subsystems.SwerveModule.BrakeMode;
import frc.robot.subsystems.SwerveModule.ModPosition;
import frc.robot.subsystems.SwerveModule.Motor;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private SwerveModule[] _swerveModules; 
  private SwerveDriveOdometry _swerveDriveOdometry; 
  private SwerveDrivePoseEstimator _swervePoseEstimator; 
  private Field2d _field = new Field2d(); 
  private ADIS16470_IMU _imu = new ADIS16470_IMU(); 
  private ADIS16470_IMUSim _imuSim = new ADIS16470_IMUSim(_imu); 
  private StructArrayPublisher<SwerveModuleState> _swerveTargetStatesPub; 
  private StructArrayPublisher<SwerveModuleState> _swerveStatesPub;
  private StructPublisher<Pose2d> _posePub; 
  private StructPublisher<Pose2d> _targetPub; 
  private StructPublisher<ChassisSpeeds> _speedPub; 
  private double _lastTime;

  public boolean isAutoRotating = false;
  private double targetAngle = 0;

  public double getTargetAngle() {
    return targetAngle;
  }

  public void setTargetAngle(double angle) {
    targetAngle = angle; 
    _autoRotatePID.setSetpoint(angle);
  }
  private PIDController _autoRotatePID;

  public enum RobotSide {
    kShooter, 
    kIntake
  }
  
  private RobotSide _front; 
  public DriveTrain() {
    this.setName("Drive Train");

    _swerveModules = new SwerveModule[] {
      new SwerveModule(ModPosition.FRONT_LEFT),
      new SwerveModule(ModPosition.FRONT_RIGHT),
      new SwerveModule(ModPosition.BACK_LEFT),
      new SwerveModule(ModPosition.BACK_RIGHT)
    };

    SmartDashboard.putNumber("Drive/AutoRotate/P", 0.002);
    SmartDashboard.putNumber("Drive/AutoRotate/I", 0.0);
    SmartDashboard.putNumber("Drive/AutoRotate/D", 0.0001);
    _autoRotatePID = new PIDController(0.002, 0.0, 0.0001);

    SmartDashboard.putData("Drive/AutoRotate/Update", new InstantCommand(() -> {
      double P = SmartDashboard.getNumber("Drive/AutoRotate/P", 0.002);
      double I = SmartDashboard.getNumber("Drive/AutoRotate/I", 0.0);
      double D = SmartDashboard.getNumber("Drive/AutoRotate/D", 0.0001);

      System.out.println("Set PID to " + P + ", " + I + ", " + D);

      _autoRotatePID.setPID(P, I, D);
    }));

    _autoRotatePID.enableContinuousInput(-180, 180);
    _autoRotatePID.setTolerance(10.0,0.1);

    // set up Odometry and Pose
    _swerveDriveOdometry = new SwerveDriveOdometry(Constants.SwerveChassis.SWERVE_KINEMATICS,
    new Rotation2d(_imu.getAngle(IMUAxis.kYaw )),
    getPositions()); 
    
    _swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveChassis.SWERVE_KINEMATICS, Rotation2d.fromDegrees(0), getEstimatedPosition(), new Pose2d(1, 3, Rotation2d.fromDegrees(0))); 
    AutoBuilder.configureHolonomic(
      this::getPoseEstimate, 
      this::resetPoseEstimator,
      this::getRobotRelativeSpeeds,
      this::driveRobotRelative,
      Constants.SwerveChassis.kFollowerConfig,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    },
    this); 
    PathPlannerLogging.setLogActivePathCallback((poses) -> _field.getObject("path").setPoses(poses));
    // Set up Publishers 
    _swerveTargetStatesPub = NetworkTableInstance.getDefault().getStructArrayTopic("TargetStates", SwerveModuleState.struct).publish(); 
    _swerveStatesPub = NetworkTableInstance.getDefault().getStructArrayTopic("States", SwerveModuleState.struct).publish();
    _posePub = NetworkTableInstance.getDefault().getStructTopic("Pose", Pose2d.struct).publish(); 
    _targetPub = NetworkTableInstance.getDefault().getStructTopic("Target Pose", Pose2d.struct).publish(); 
    _speedPub = NetworkTableInstance.getDefault().getStructTopic("Chassis Speeds", ChassisSpeeds.struct).publish();
    _lastTime = Timer.getFPGATimestamp();  
    _front = RobotSide.kIntake; 
    
  }
  private SwerveModulePosition[] getEstimatedPosition() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < positions.length; i++){
      positions[i] = _swerveModules[i].getEstimatedPosition(); 
    }
    return positions;
  }
  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < positions.length; i++){
      positions[i] = _swerveModules[i].getPosition(); 
    }
    return positions; 
  }

  public void setGyro(double angle) {
    _imu.setGyroAngle(IMUAxis.kYaw, angle);
  }

  public double getAngle() {
    return normalizeAngle(getRawAngle());
  }

  public double getRawAngle() {
    return _imu.getAngle(IMUAxis.kYaw);
  }

  private double normalizeAngle(double angle) {
    while(angle <= -180) angle += 360;
    while(angle > 180) angle -= 360;
    return angle;
  }

  public double realDistance(double to) {
    double from = normalizeAngle(getAngle());
    to = normalizeAngle(to);

    double diff = to - from;
    if(diff <= -180) diff += 360;
    else if(diff > 180) diff -= 360;

    return diff;
  }

  public boolean isNear(double angle) {
    return Math.abs(getAngle() - angle) < 2;
  }

  public void resetOdometry(Pose2d pose) {
    _swerveDriveOdometry.resetPosition(new Rotation2d(_imu.getAngle(IMUAxis.kYaw)), getPositions(), pose);
  }
  public Pose2d getPose2d() {
    return _swerveDriveOdometry.getPoseMeters(); 
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[_swerveModules.length];
    for (int i = 0; i< swerveModuleStates.length; i++) {
      swerveModuleStates[i] = _swerveModules[i].getState(); 
    }
    return Constants.SwerveChassis.SWERVE_KINEMATICS.toChassisSpeeds(swerveModuleStates); 
  }
  public void resetPoseEstimator(Pose2d pose) {
    _swervePoseEstimator.resetPosition(new Rotation2d(_imu.getAngle(IMUAxis.kYaw)), getPositions(), pose);
  }
  public Pose2d getPoseEstimate() {
    return _swervePoseEstimator.getEstimatedPosition();
  }
  public void setToStartPos(){
    _swervePoseEstimator.resetPosition(Rotation2d.fromDegrees(_imu.getAngle(IMUAxis.kYaw)), getPositions(), new Pose2d(1.50, 5.54, Rotation2d.fromDegrees(0)));
    _swerveDriveOdometry.resetPosition(Rotation2d.fromDegrees(_imu.getAngle(IMUAxis.kYaw)), getPositions(), new Pose2d(1.50, 5.54, Rotation2d.fromDegrees(0)));
  }

  public void driveRobotRelative(ChassisSpeeds speed){
    SwerveModuleState[] swerveModuleStates;
    swerveModuleStates = SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(
        speed
    ); 
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveChassis.kSuperEpicTurboMaxVelocity);
    for (int i = 0; i<_swerveModules.length; i++) {
      _swerveModules[i].setDesiredState(swerveModuleStates[i]);
    }
  }
  public void setFrontOfRobot(RobotSide front)
  {
    // if We arent changing anything, just leave 
    if (_front == front) return; 
    Pose2d pose = new Pose2d(getPose2d().getX(), getPose2d().getY(), Rotation2d.fromDegrees(0)); 
    Pose2d targetPose2d = new Pose2d(getPoseEstimate().getX(), getPoseEstimate().getY(), getPoseEstimate().getRotation().minus(Rotation2d.fromDegrees(0))); 
    resetOdometry(pose);
    resetPoseEstimator(targetPose2d);
    // set new state. 
    _front = front; 
  }
  public RobotSide getFrontOfRobot()
  {
    return _front; 
  }


  public void setIsAutoRotating(boolean shouldAutoRotate) {
    this.isAutoRotating = shouldAutoRotate;
  }

  private double targetDelta = 0.0;

  public double getTargetDelta() {
    return targetDelta;
  }

  public void setTargetDelta(double delta) {
    targetDelta = delta;
  }

  public double vel_x;
  public double vel_y;

  // public double magnitude;

  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s, boolean fieldcentric){
    SwerveModuleState[] swerveModuleStates;

    // double inputMagnitude = Math.sqrt(xVelocity_m_per_s * xVelocity_m_per_s + yVelocity_m_per_s * yVelocity_m_per_s);

    // magnitude += 0.05 * (inputMagnitude - magnitude);

    //System.out.println("***X: "+xVelocity_m_per_s+" ***Y: "+yVelocity_m_per_s+" ***o: "+omega_rad_per_s);
    if (fieldcentric) { // field-centric swerve
      double angleDelta = 0; 
      if (isAutoRotating){
        angleDelta = _autoRotatePID.calculate(this.getAngle());
      } 
      
      SmartDashboard.putBoolean("Drive/AutoRotating", isAutoRotating);
      if(!isAutoRotating) {
        targetDelta = omega_rad_per_s;
      }
      SmartDashboard.putNumber("Drive/Target Delta", targetDelta);

      swerveModuleStates = SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xVelocity_m_per_s,// * magnitude,
              yVelocity_m_per_s,// * magnitude,
              (isAutoRotating) ? angleDelta : omega_rad_per_s,
              Rotation2d.fromDegrees(_imu.getAngle(IMUAxis.kYaw))));
    } else { // robot-centric swerve; does not use IMU
      swerveModuleStates = SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(
          new ChassisSpeeds(
              xVelocity_m_per_s,// * magnitude,
              yVelocity_m_per_s,// * magnitude,
              omega_rad_per_s));
    }
    // Evaluate if auto turning is done. If so turn it off. 
    if (isNear(getTargetAngle()) || Math.abs(omega_rad_per_s) > 0.0)
    {
      setIsAutoRotating(false);
    }
    SmartDashboard.putBoolean("Drive/Field Centric", fieldcentric);
    SmartDashboard.putNumber("Drive/Angle", getAngle());
    SmartDashboard.putNumber("Drive/Rotation", new Rotation2d(_imu.getAngle(IMUAxis.kYaw)).getDegrees());
    SmartDashboard.putNumber("Drive/Angle Raw", _imu.getAngle(IMUAxis.kYaw));
    // Because the resulting power is a vector sum of the robot direction and
    // rotation, it's possible that
    // the resulting vector would exceed absolute scalar value of 1. And we need to
    // keep the power in the -1..+1 range.
    // Hence if any of the vectors have a scalar value greater than 1, we need to
    // divide all of them by the largest scalar value
    // of a vector we have. That will preserve the direction of the robot even it is
    // concurrently combioned with the
    // holonomic rotation.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveChassis.kSuperEpicTurboMaxVelocity);
    int i = 0; 
    for (SwerveModule mod : _swerveModules) {
      mod.setDesiredState(swerveModuleStates[i++]);
    }
  }
  
  public void simulationInit()
  {
    for (SwerveModule mod : _swerveModules) {
      mod.simulationInit();
    }
  }
  @Override
  public void simulationPeriodic(){
    // update the Gyro based on chassis speeds. 
    ChassisSpeeds speeds = Constants.SwerveChassis.SWERVE_KINEMATICS.toChassisSpeeds(getTargetStates()); 
    
    _imuSim.setGyroAngleZ(_imu.getAngle(IMUAxis.kYaw)+Math.toDegrees(speeds.omegaRadiansPerSecond)*(Timer.getFPGATimestamp() - _lastTime));
    _lastTime = Timer.getFPGATimestamp(); 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    _swervePoseEstimator.update(Rotation2d.fromDegrees(_imu.getAngle(IMUAxis.kYaw)), getEstimatedPosition()); 
    _swerveDriveOdometry.update(Rotation2d.fromDegrees(_imu.getAngle(IMUAxis.kYaw)), getPositions()); 
    _swerveTargetStatesPub.set(getTargetStates());
    _swerveStatesPub.set(getStates()); 
    _posePub.set(getPose2d());
    _targetPub.set(_swervePoseEstimator.getEstimatedPosition()); 
    _speedPub.set(Constants.SwerveChassis.SWERVE_KINEMATICS.toChassisSpeeds(getTargetStates())); 
    SmartDashboard.putNumber("Normalized Gyro Angle", getAngle()); 
    SmartDashboard.putNumber("Raw Gyro Angle", getRawAngle()); 
    
  }
   /**
   * This method updates swerve odometry. Note that we do not update it in a
   * periodic method
   * to reduce the CPU usage, since the odometry is only used for trajectory
   * driving.
   * If you use odometry for other purposes, such as using vision to track game
   * elements,
   * either update the odometry in periodic method, or update it from appropriate
   * commands.
   * 
   * The telemetry will print the IMU and swerve positions we send to odometry
   * along with
   * the same information obtained from telemetry after update.
   * It may help troubleshooting potential odometry update issues (e.g. units of
   * measure issues etc)
   * 
   * Please, note that use of telemetry may significantly increase CPU usage, and
   * can ultimately result
   * in a packet loss. We recommend disabling excessive telemetry for the
   * competition.
   */
  public void updateTrajectoryOdometry() {
    
      _swerveDriveOdometry.update(new Rotation2d(_imu.getAngle(IMUAxis.kYaw)), getPositions());
  }
  
  public void testSwervePods()
  {
    for (SwerveModule mod: _swerveModules)
    {
      mod.testAngleMotor(0.3);
    }
  }
  public void testAnglePositions(double angle){
    for (SwerveModule mod : _swerveModules)
    {
      mod.testAnglePosition(angle);
    }
  }
  public void setDriveBrakeMode(boolean brake){
    for (SwerveModule mod: _swerveModules)
    {
      mod.setBrakeMode(Motor.DRIVE, (brake) ? BrakeMode.BRAKE : BrakeMode.COAST);;
    }
  }
  public void stopMotors(){
    
    drive(0, 0, 0, false);
  }
  public boolean flipPath(){
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? false : true; 
  }
  public SwerveModuleState[] getTargetStates(){
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[_swerveModules.length];
    for (int i = 0; i< swerveModuleStates.length; i++) {
      swerveModuleStates[i] = _swerveModules[i].getTargetState(); 
    }
    return swerveModuleStates; 
  }
  public SwerveModuleState[] getStates(){
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[_swerveModules.length];
    for (int i = 0; i< swerveModuleStates.length; i++) {
      swerveModuleStates[i] = _swerveModules[i].getState(); 
    }
    return swerveModuleStates; 
  }

  public void turnToFieldElement(Constants.GameConstants.FieldElement element) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue); 
    setTargetAngle(NavUtilites.angleForFieldElement(element,alliance ));
    setIsAutoRotating(true);
  }

  public void turnToAngle(double angle) {
    setTargetAngle(angle);
    setIsAutoRotating(true);
  }
  
}