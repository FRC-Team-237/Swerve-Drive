// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class NavUtilites {

    public static Command makePath(String pathFile, DriveTrain driveTrain) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathFile);
        FollowPathHolonomic pathFollower = new FollowPathHolonomic(path, driveTrain::getPose2d,
                driveTrain::getRobotRelativeSpeeds, driveTrain::driveRobotRelative,
                Constants.SwerveChassis.kFollowerConfig, driveTrain::flipPath, (Subsystem) driveTrain);
        return pathFollower;
    }
    public static double angleForFieldElement(Constants.GameConstants.FieldElement element, Alliance alliance) {
        switch (element){
            case kSpeaker:
                return 0.0; 
            case kSource:
                return (alliance == Alliance.Blue) ? Constants.GameConstants.kBlueSourceAngle : Constants.GameConstants.kRedSourceAngle; 
            case kAmp: 
                return (alliance == Alliance.Blue) ? Constants.GameConstants.kBlueAmpAngle : Constants.GameConstants.kRedAmpAngle;
            case kStage: 
                return (alliance == Alliance.Blue) ? Constants.GameConstants.kBlueStageAngle : Constants.GameConstants.kRedStageAngle;
          }
        return 0.0; 
    }
}
