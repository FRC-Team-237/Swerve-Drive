// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

public static class SwerveChassis {

    public static final double kADrive = 0;
    public static double kVDrive;
    public static double kSDrive;
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


}
