// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Utilities.PathUtilities;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Action;
import frc.robot.subsystems.IntakeSubsystem.Action;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  public final DriveTrain _drive = new DriveTrain(); 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kXboxControllerPort);
  private final Joystick _logitechJoystick = new Joystick(OperatorConstants.kLogitechControllerPort);
  private final JoystickButton _button = new JoystickButton(_logitechJoystick, 1);
  private final Joystick _3AxisJoystick = new Joystick(1);

  private final JoystickButton _resetPose = new JoystickButton(_logitechJoystick, 2); 

  public final IntakeSubsystem _intake = new IntakeSubsystem();
  private final HangerSubsystem _hanger = new HangerSubsystem();
  private final Map<String,Command> _commandMap = new HashMap<String,Command>(); 
  private boolean fieldCentric;
  private final ShooterSubsystem _shooter = new ShooterSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    SmartDashboard.putString("Path to Test", "Test Path");
    setupNamedCommands();
    configureBindings();

    _drive.setDefaultCommand((new RunCommand(() -> {
      double velocityX = -m_driverController.getLeftY() * Constants.SwerveChassis.kMaxVelocity;
      double velocityY = -m_driverController.getLeftX() * Constants.SwerveChassis.kMaxVelocity;

      double rot = m_driverController.getRightX();

      // if(_3AxisJoystick.isConnected()) {
      //   velocityX = -_3AxisJoystick.getX() * Constants.SwerveChassis.kMaxVelocity;
      //   velocityY = _3AxisJoystick.getY() * Constants.SwerveChassis.kMaxVelocity;
      //   rot = _3AxisJoystick.getZ();
      // }

      rot = Math.abs(rot) > 0.2 ? rot : 0;
      rot /= 4.0;
      rot *= Math.abs(rot);
      
      rot *= 2.0 * Math.PI / 12.0;

      velocityX = Math.abs(velocityX) > 0.12 ? velocityX : 0;
      velocityY = Math.abs(velocityY) > 0.12 ? velocityY : 0;

      // SmartDashboard.putNumber("Rotation speed", rot);

      _drive.drive(
        velocityX * Math.abs(velocityX),
        velocityY * Math.abs(velocityY),
        rot,
        fieldCentric
      );
    }, _drive)));
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
    // Test path following. 
    // m_driverController.b()
    //   .onTrue(new InstantCommand(() -> {
    //     String pathName = SmartDashboard.getString("Path to Test", "Test Path"); 
    //     PathUtilities.makePath(pathName, _drive).schedule(); 
    //   }))
    //   .onFalse(new InstantCommand(_drive::stopMotors, _drive)); 
    
    // m_driverController.y()
    //   .onTrue(new InstantCommand(() -> {
    //     fieldCentric = !fieldCentric;
    //     System.out.println("Toggled field centric");
    //   }));

    // m_driverController.povDown()
    //   .onTrue(new InstantCommand(() -> {
    //     _drive.resetOdometry(new Pose2d(1.90, 5.32, Rotation2d.fromDegrees(0)));
    //     String pathName = SmartDashboard.getString("Path to Test", "Test Path"); 
    //     PathUtilities.makePath(pathName, _drive).schedule();
    //   }))
    //   .onFalse(new InstantCommand(_drive::stopMotors, _drive));
      // .onFalse(new InstantCommand(() -> {
      //   _drive.stopMotors();
      // }));

    // m_driverController.povUp()
    //   .whileTrue(new TestFollowCommand());

    m_driverController.rightTrigger(0.1)
      .onTrue(_commandMap.get("ShootCommand"))
      .onFalse(new InstantCommand(() -> {
        _shooter.stopShoot();
        _shooter.stopFeed();
        _intake.stopIntakeMotor();
       },_shooter,_intake));
    
    m_driverController.rightBumper()
      .onTrue(new InstantCommand(_shooter::intake))
      .onFalse(new InstantCommand(_shooter::stopIntake));
    
    m_driverController.leftTrigger(0.1)
      .onTrue(new InstantCommand(_shooter::spit)
      .andThen(new InstantCommand(_shooter::feed)
      .andThen(_intake.getActionCommand(Action.FIRE)))
      )
      .onFalse(new InstantCommand(()->{
        _shooter.stopFeed();
        _shooter.stopShoot();
        _intake.stopIntakeMotor();
      },_shooter,_intake));
      
    
    
    // m_driverController.povDown()
    //   .onTrue(new InstantCommand(() -> _intake.movePositionMotor(0.2)))
    //   .onFalse(new InstantCommand(() -> _intake.movePositionMotor(0)));
    
    m_driverController.povUp()
      .onTrue(new InstantCommand(() -> _intake.movePositionMotor(-0.2)))
      .onFalse(new InstantCommand(() -> _intake.movePositionMotor(0)));

    m_driverController.a()
      .onTrue(
        _commandMap.get("PickUpCommand")
      ); 
      
    m_driverController.b()
    .onTrue(_intake.getActionCommand(Action.LOAD).andThen(() -> _shooter.stopFeed(),_shooter));
    m_driverController.y()
      .onTrue(new InstantCommand(_hanger::extend))
      .onFalse(new InstantCommand(_hanger::stop));
      
    m_driverController.x()
    .onTrue(new InstantCommand(_hanger::retract))
    .onFalse(new InstantCommand(_hanger::stop));
      
     

  }

  public Command getTestPathCommand() {
    String pathName = SmartDashboard.getString("Path to Test", "Test Path"); 
    System.out.println(pathName);
    return PathUtilities.makePath(pathName, _drive); 
  }

  /**
 * Sets up the named commands for the robot.
 * 
 * This method creates and registers several named commands for the robot. The named commands include:
 * - TargetCommand: A command that does nothing and is used as a target for other commands.
 * - ShootCommand: A command that shoots balls. It first calls the shoot method of the shooter subsystem, then waits for 5 seconds, checks if the shooter is at speed, and finally feeds balls into the shooter and sets the intake motor to -0.75.
 * - PickUpCommand: A command that picks up notes. It first calls the intake action command with the INTAKE action, then calls the intake action command with the LOAD action.
 * - EjectCommand: A command that ejects notes. It calls the intake action command with the EJECT action.
 * 
 * After creating and naming the commands, they are registered with the NamedCommands class for use with the path planner.
 */
  public void setupNamedCommands(){
    
    // Target command 
    Command targetCommand = new InstantCommand(); 
    targetCommand.setName("TargetCommand");
    _commandMap.put(targetCommand.getName(),targetCommand); 
    
    // shoot command 
    Command shootCommand = new InstantCommand(()->{
          _shooter.shoot();
        },_shooter)
        // TODO: Investigate this The AI seems to think this will wait for 5 seconds regardless of if the shooter is up to speed. 
        .andThen(new WaitCommand(5))
        .until(_shooter::atSpeed)
        .andThen(()->{
          _shooter.feed();
          _intake.setIntakeMotor(-0.75);
        },_shooter,_intake); 
    shootCommand.setName("ShootCommand");
    _commandMap.put(shootCommand.getName(),shootCommand); 
    
    // Command to Pick up the notes
    Command pickUpCommand = _intake.getActionCommand(Action.INTAKE)
        .andThen(_intake.getActionCommand(Action.LOAD)); 
    pickUpCommand.setName("PickUpCommand");
    _commandMap.put(pickUpCommand.getName(),pickUpCommand);
    // Command to Eject notes 
    Command ejectCommand = _intake.getActionCommand(Action.EJECT); 
    ejectCommand.setName("EjectCommand");
    _commandMap.put(ejectCommand.getName(), ejectCommand); 
    

    // Register all Named commands for use with path planner. 
    _commandMap.forEach((key,value)->{
      NamedCommands.registerCommand(key, value);
    });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }
}
