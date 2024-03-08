// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Action;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class CommandFactory {
    public static Command makeShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake){
        
        return new RunCommand(()->{
          shooter.shoot();
        },shooter) 
        .until(shooter::atSpeed)
        .withTimeout(2)
        .andThen(()->{
          shooter.feed();
          intake.setIntakeMotor(-0.75);
        },shooter,intake);  
    }
    public static Command makePickUpCommand(IntakeSubsystem intake){
        return intake.getActionCommand(Action.INTAKE)
        .andThen(intake.getActionCommand(Action.LOAD));
    }
    public static Command makeEject(IntakeSubsystem intake){
        return intake.getActionCommand(Action.EJECT); 
    }
    
}
