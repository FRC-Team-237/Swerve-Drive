// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.SwerveChassis;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveModule.ModPosition;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private SwerveModule[] _swerveModules; 
  private SwerveDriveOdometry _swerveDriveOdometry; 
  private SwerveDrivePoseEstimator _swervePoseEstimator; 
  private Field2d _field = new Field2d(); 
  private ADIS16470_IMU _imu = new ADIS16470_IMU(); 


  public DriveTrain() {
    this.setName("Drive Train");

    _swerveModules = new SwerveModule[] {
      new SwerveModule(ModPosition.FRONT_LEFT),
      new SwerveModule(ModPosition.FRONT_RIGHT),
      new SwerveModule(ModPosition.BACK_LEFT),
      new SwerveModule(ModPosition.BACK_RIGHT)
    }; 

    // set up Odometry and Pose
    _swerveDriveOdometry = new SwerveDriveOdometry(Constants.SwerveChassis.SWERVE_KINEMATICS,
    new Rotation2d(_imu.getAngle(_imu.getYawAxis())),
    getPositions()); 

    AutoBuilder.configureHolonomic(
      this::getPose2d, 
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
  }
  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < positions.length; i++){
      positions[i] = _swerveModules[i].getPosition(); 
    }
    return positions; 
  }

  public void resetOdometry(Pose2d pose)
  {
    _swerveDriveOdometry.resetPosition(new Rotation2d(_imu.getAngle(_imu.getYawAxis())), getPositions(), pose);
  }
  public Pose2d getPose2d()
  {
    return _swerveDriveOdometry.getPoseMeters(); 
  }

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    SwerveModuleState[] swerveModuleStates = new SwerveModuleState[_swerveModules.length];
    for (int i = 0; i< swerveModuleStates.length; i++)
    {
      swerveModuleStates[i] = _swerveModules[i].getState(); 
    }
    return Constants.SwerveChassis.SWERVE_KINEMATICS.toChassisSpeeds(swerveModuleStates); 
  }
  public void resetPoseEstimator(Pose2d pose) {
    _swervePoseEstimator.resetPosition(new Rotation2d(_imu.getAngle(_imu.getYawAxis())), getPositions(), pose);
  }
  public Pose2d getPoseEstimate() {
    return _swervePoseEstimator.getEstimatedPosition();
  }

  public void driveRobotRelative(ChassisSpeeds speed){
    SwerveModuleState[] swerveModuleStates;
    swerveModuleStates = SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(
        speed
    ); 
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveChassis.kMaxVelocity);
    for (int i = 0; i<_swerveModules.length; i++) {
      _swerveModules[i].setDesiredState(swerveModuleStates[i]);
    }
  }
  public void drive(double xVelocity_m_per_s, double yVelocity_m_per_s, double omega_rad_per_s, boolean fieldcentric){
    SwerveModuleState[] swerveModuleStates;
    //System.out.println("***X: "+xVelocity_m_per_s+" ***Y: "+yVelocity_m_per_s+" ***o: "+omega_rad_per_s);
    if (fieldcentric) { // field-centric swerve
      swerveModuleStates = SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xVelocity_m_per_s,
              yVelocity_m_per_s,
              omega_rad_per_s,
              Rotation2d.fromDegrees(_imu.getAngle(_imu.getYawAxis()))));
    } else { // robot-centric swerve; does not use IMU
      swerveModuleStates = SwerveChassis.SWERVE_KINEMATICS.toSwerveModuleStates(
          new ChassisSpeeds(
              xVelocity_m_per_s,
              yVelocity_m_per_s,
              omega_rad_per_s));
    }

    // Because the resulting power is a vector sum of the robot direction and
    // rotation, it's possible that
    // the resulting vector would exceed absolute scalar value of 1. And we need to
    // keep the power in the -1..+1 range.
    // Hence if any of the vectors have a scalar value greater than 1, we need to
    // divide all of them by the largest scalar value
    // of a vector we have. That will preserve the direction of the robot even it is
    // concurrently combioned with the
    // holonomic rotation.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveChassis.kMaxVelocity);
    int i = 0; 
    for (SwerveModule mod : _swerveModules) {
      mod.setDesiredState(swerveModuleStates[i++]);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    
      _swerveDriveOdometry.update(new Rotation2d(_imu.getAngle(_imu.getYawAxis())), getPositions());
  }
  
}
