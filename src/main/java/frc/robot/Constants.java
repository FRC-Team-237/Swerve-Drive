// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final LoggingType logLevel = LoggingType.SMART_DASH; 
  public static enum LoggingType {
    NO_LOGGING,
    SMART_DASH,
    PRINT, 
    SMART_DASH_AND_PRINT
  }; 
  public static class OperatorConstants {
    public static final int kXboxControllerPort = 5;
    public static final int kLogitechControllerPort = 0;
  }
public static class IntakeConstants {
  public static final int kDeployMotorId = 23; 
  public static final int kIntakeMotorId = 0; 
  public static final double kDeployedPos = 41.6;
  public static final double kRetractedPos = 0.0;
public static int kGamePieceSensorPort = 0; 
}
public static class SwerveChassis {

    public static final double kADrive = 0;
    public static final double kPDrive = 0.11;
    public static final double kIDrive = 0.5;
    public static final double kDDrive = 0.0001;
    public static final double kVDrive =0.11;
    public static final double kSDrive = 0.0;
    public static final double kPeakForwardFF = 8; 
    public static final double kPeakReverseFF = -8;
    private static final double TRACK_WIDTH = 22;
    private static final double WHEEL_BASE = 22; 
    // max speed in meters per second. 
    public static double kWheelRadius = Units.inchesToMeters(2.0);  
    public static double kMaxVelocity = 2.0; 
    public static double kMaxRotationsPerSec = kMaxVelocity/(2*Math.PI*kWheelRadius);
    public static double degreesPerRotation = 30.0;
    public static double degreesPerTick = 360.0/degreesPerRotation;  
    public static double kDriveGearRatio = 6.75; 
    public static final double kAngleGearRatio = 12.8; // 12 rotations of motor for one rotation of the wheel. 
    public static final double kAngleConversionFactor = 360.0 / kAngleGearRatio;  
    public static double metersPerRev = kWheelRadius*2*Math.PI; 

    public static final HolonomicPathFollowerConfig kFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(5.0, 0.0, 0.0),
      new PIDConstants(5.0, 0.0, 0.0),
      kMaxVelocity,
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0).getNorm(),
      new ReplanningConfig()
    );

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
				new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    // Constants for swerve pods. 
    public static class FrontLeft {
      public static int kDriveId = 17;
      public static int kAngleId = 16; 
      public static boolean kDriveInverted = false; 
      public static boolean kAngleInverted = false; 
      public static boolean kDriveSensorPhaseInverted = false; 
    }  

    public static class FrontRight {
      public static int kDriveId = 15;
      public static int kAngleId = 14; 
      public static boolean kDriveInverted = false; 
      public static boolean kAngleInverted = false; 
      public static boolean kDriveSensorPhaseInverted = false; 
    }  

    public static class BackLeft {
      public static int kDriveId = 11;
      public static int kAngleId = 10; 
      public static boolean kDriveInverted = false; 
      public static boolean kAngleInverted = false; 
      public static boolean kDriveSensorPhaseInverted = false; 
    }  
    
    public static class BackRight {
      public static int kDriveId = 13;
      public static int kAngleId = 12; 
      public static boolean kDriveInverted = false; 
      public static boolean kAngleInverted = false; 
      public static boolean kDriveSensorPhaseInverted = false; 
    }  
}

  public static class Mechanism {
    public static int kHangerMotorId = 9;

    public static int kShooterLowMotorId = 20;
    public static int kShooterHighMotorId = 21;
    public static int kShooterFeederMotorId = 22;

    public static double kShooterMaxTargetRPM = 5200;
    // public static double kShooterMaxTargetRPM = 1.0;
    public static double kIntakeMultiplier = 0.15;
    public static double kSpitMultiplier = 0.2;
    public static double kShooterFeedMultiplier = 0.9;
  }
}
