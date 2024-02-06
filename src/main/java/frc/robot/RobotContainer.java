// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Utilities.PathUtilities;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
// import frc.robot.commands.TestFollowCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final DriveTrain _drive = new DriveTrain(); 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kXboxControllerPort);
  private final Joystick _logitechJoystick = new Joystick(OperatorConstants.kLogitechControllerPort);
  private final JoystickButton _button = new JoystickButton(_logitechJoystick, 1);
  private final Joystick _3AxisJoystick = new Joystick(1);

  private boolean fieldCentric;
  private final ShooterSubsystem _shooter = new ShooterSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    SmartDashboard.putString("Path to Test", "Test Path"); 
    configureBindings();
    _drive.setDefaultCommand((new InstantCommand(() -> {
      double shootPower = m_driverController.getRightTriggerAxis() * Constants.Mechanism.kShooterMaxTargetRPM;
      // power -= m_driverController.getLeftTriggerAxis() * Constants.Mechanism.kIntakeMultiplier;

      if(m_driverController.leftBumper().getAsBoolean()) {
        shootPower = -Constants.Mechanism.kIntakeMultiplier * Constants.Mechanism.kShooterMaxTargetRPM;
      } else if(m_driverController.rightBumper().getAsBoolean()) {
        shootPower = Constants.Mechanism.kSpitTargetRPM;
      }

      _shooter.output(shootPower);

      // double feedPower = m_driverController.getLeftTriggerAxis() * Constants.Mechanism.kShooterFeedMultiplier;
      // double feedPower = m_driverController.getLeftTriggerAxis() > 0.5 ? Constants.Mechanism.kShooterFeedMultiplier : 0.0;

      boolean feed = m_driverController.a().getAsBoolean();
      double feedPower = feed ? 0.5 : 0.0;

      SmartDashboard.putBoolean("Shooter/Feed", feed);

      _shooter.feed(feedPower);

      double velocityX = -m_driverController.getLeftY() * Constants.SwerveChassis.kMaxVelocity;
      double velocityY = -m_driverController.getLeftX() * Constants.SwerveChassis.kMaxVelocity;
      double rot = m_driverController.getRightX();

      if(_3AxisJoystick.isConnected()) {
        velocityX = -_3AxisJoystick.getX() * Constants.SwerveChassis.kMaxVelocity;
        velocityY = _3AxisJoystick.getY() * Constants.SwerveChassis.kMaxVelocity;
        rot = _3AxisJoystick.getZ();
      }

      rot = Math.abs(rot) > 0.2 ? rot : 0;
      rot /= 4.0;
      rot *= Math.abs(rot);
      
      rot *= 2.0 * Math.PI / 12.0;

      velocityX = Math.abs(velocityX) > 0.1 ? velocityX : 0;
      velocityY = Math.abs(velocityY) > 0.1 ? velocityY : 0;

      SmartDashboard.putNumber("Rotation speed", rot);

      _drive.drive(
        velocityX * Math.abs(velocityX),
        velocityY * Math.abs(velocityY),
        rot,
        fieldCentric
      );
    }, _drive, _shooter)).repeatedly());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Test path following. 
    m_driverController.b()
      .onTrue(new InstantCommand(() -> {
        String pathName = SmartDashboard.getString("Path to Test", "Test Path"); 
        PathUtilities.makePath(pathName, _drive).schedule(); 
      }))
      .onFalse(new InstantCommand(_drive::stopMotors, _drive)); 
    
    m_driverController.y()
      .onTrue(new InstantCommand(() -> {
        fieldCentric = !fieldCentric;
        System.out.println("Toggled field centric");
      }));
    
    m_driverController.x().onTrue(new InstantCommand(() -> {
      _shooter.feed(0.5);
    })).onFalse(new InstantCommand(() -> {
      _shooter.feed(0.0);
    }));

    m_driverController.povDown()
      .onTrue(new InstantCommand(() -> {
        _drive.resetOdometry(new Pose2d(1.90, 5.32, Rotation2d.fromDegrees(0)));
        String pathName = SmartDashboard.getString("Path to Test", "Test Path"); 
        PathUtilities.makePath(pathName, _drive).schedule();
      }))
      .onFalse(new InstantCommand(_drive::stopMotors, _drive));
      // .onFalse(new InstantCommand(() -> {
      //   _drive.stopMotors();
      // }));

    // m_driverController.povUp()
    //   .whileTrue(new TestFollowCommand());
  }

  public Command getTestPathCommand() {
    String pathName = SmartDashboard.getString("Path to Test", "Test Path"); 
    System.out.println(pathName);
    return PathUtilities.makePath(pathName, _drive); 
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
