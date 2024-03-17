// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.GameConstants.FieldElement;
import frc.robot.Utilities.CommandFactory;
import frc.robot.Utilities.NavUtilites;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.JustShootAuto;
import frc.robot.commands.MoveIntakeCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Action;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveTrain _drive = new DriveTrain();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kXboxControllerPort);
  private final Joystick _buttonPanel = new Joystick(OperatorConstants.kButtonPanelPort);
  public final IntakeSubsystem _intake = new IntakeSubsystem();
  private final HangerSubsystem _hanger = new HangerSubsystem();
  private boolean fieldCentric = true;
  private final ShooterSubsystem _shooter = new ShooterSubsystem();
  private SendableChooser<Command> m_chooser = new SendableChooser<>(); 
  public enum CommandType {
    kShoot,
    kPickup,
    kLoad,
    kTarget
  }
  public enum StartingPosition {
    kCenter,
    kLeft,
    kRight
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    SmartDashboard.putString("Path to Test", "Test Path");
    m_chooser.setDefaultOption("Only Shoot", makeShootCommand());
    m_chooser.addOption("Left One Note", makeSideAutoCommand(true));
    m_chooser.addOption("Right One Note", makeSideAutoCommand(false));
    m_chooser.addOption("Center Two Note", makeCenterAutoCommand());
    m_chooser.addOption("Center Three Note", make3NoteAutoCommand());
    configureBindings();

    _drive.setDefaultCommand((new RunCommand(() -> {

      double velocityX = (Math.abs(m_driverController.getLeftY()) > 0.1 ? -m_driverController.getLeftY() : 0);
      double velocityY = (Math.abs(m_driverController.getLeftX()) > 0.1 ? -m_driverController.getLeftX() : 0);

      velocityX *= Math.abs(velocityX);
      velocityY *= Math.abs(velocityY);

      // * Constants.SwerveChassis.kMaxVelocity
      // * Constants.SwerveChassis.kMaxVelocity

      double rot = m_driverController.getRightX();
      rot = Math.abs(rot) > 0.2 ? rot : 0;
      rot *= Math.abs(rot) * Math.abs(rot) * Math.abs(rot);
      rot *= Constants.SwerveChassis.kMaxRotationalVelocity;
      // if(_3AxisJoystick.isConnected()) {
      // velocityX = -_3AxisJoystick.getX() * Constants.SwerveChassis.kMaxVelocity;
      // velocityY = _3AxisJoystick.getY() * Constants.SwerveChassis.kMaxVelocity;
      // rot = _3AxisJoystick.getZ();
      // }

      // SmartDashboard.putNumber("Rotation speed", rot);

      double maxVelocity = m_driverController.leftBumper().getAsBoolean()
          ? Constants.SwerveChassis.kSuperEpicTurboMaxVelocity
          : Constants.SwerveChassis.kMaxVelocity;

      SmartDashboard.putNumber("Drive/Max Velocity", maxVelocity);

      _drive.drive(
          velocityX * maxVelocity,
          velocityY * maxVelocity,
          rot,
          fieldCentric);
    }, _drive)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Test path following.
    // m_driverController.b()
    // .onTrue(new InstantCommand(() -> {
    // String pathName = SmartDashboard.getString("Path to Test", "Test Path");
    // PathUtilities.makePath(pathName, _drive).schedule();
    // }))
    // .onFalse(new InstantCommand(_drive::stopMotors, _drive));

    // m_driverController.y()
    // .onTrue(new InstantCommand(() -> {
    // fieldCentric = !fieldCentric;
    // System.out.println("Toggled field centric");
    // }));

    // m_driverController.povUp()
    // .whileTrue(new TestFollowCommand());

    m_driverController.rightTrigger(0.1)
        .whileTrue(makeShootCommand());

    m_driverController.rightBumper()
        .onTrue(new InstantCommand(_shooter::intake))
        .onFalse(new InstantCommand(_shooter::stopIntake));
    // m_driverController.rightBumper()
    // .onTrue(new InstantCommand(_shooter::intake))
    // .onFalse(new InstantCommand(_shooter::stopIntake));

    m_driverController.leftTrigger(0.1)
        .onTrue(new InstantCommand(_shooter::spit)
            .andThen(new InstantCommand(_shooter::feed)
                .andThen(_intake.getActionCommand(Action.FIRE))))
        .onFalse(new InstantCommand(() -> {
          _shooter.stopFeed();
          _shooter.stopShoot();
          _intake.stopIntakeMotor();
        }, _shooter, _intake));

    m_driverController.a()
        .onTrue(
            _intake.getActionCommand(Action.INTAKE)
                .andThen(_intake.getActionCommand(Action.LOAD)));

    m_driverController.b()
        .onTrue(_intake.getActionCommand(Action.LOAD)
            .andThen(() -> _shooter.stopFeed(), _shooter));
   
    // Auto Rotate 
    m_driverController.y()
      .onTrue(new InstantCommand(() -> _drive.turnToFieldElement(FieldElement.kSource), _drive)) 
      .onFalse(new InstantCommand(() -> _drive.setIsAutoRotating(false), _drive));

    m_driverController.x()
      .onTrue(new InstantCommand(() -> _drive.turnToFieldElement(FieldElement.kSpeaker), _drive)) 
      .onFalse(new InstantCommand(() -> _drive.setIsAutoRotating(false), _drive)); 

    // button panel
    
    if (RobotBase.isReal()) {
      // hanger retract
      new JoystickButton(_buttonPanel, 1)
          .whileTrue(new RunCommand(_hanger::retract)
            .finallyDo(_hanger::stop));

      new JoystickButton(_buttonPanel, 1)
        .and(m_driverController.back())
          .whileTrue(new RunCommand(_hanger::retractUnsafe))
          .onFalse(new InstantCommand(_hanger::stop));
          
      // hanger extend
      new JoystickButton(_buttonPanel, 2)
        .whileTrue(new RunCommand(_hanger::extend)
            .finallyDo(_hanger::stop));

      new JoystickButton(_buttonPanel, 2)
        .and(m_driverController.back())
          .whileTrue(new RunCommand(_hanger::extendUnsafe))
          .onFalse(new InstantCommand(_hanger::stop));

      // amp shoot
      new JoystickButton(_buttonPanel, 3)
          .onTrue(new InstantCommand(_shooter::spit)
              .andThen(new InstantCommand(_shooter::feed)
                  .andThen(_intake.getActionCommand(Action.FIRE))))
          .onFalse(new InstantCommand(() -> {
            _shooter.stopFeed();
            _shooter.stopShoot();
            _intake.stopIntakeMotor();
          }, _shooter, _intake));
      new JoystickButton(_buttonPanel, 4)
          .onTrue(new InstantCommand(() -> _intake.setIntakeMotor(-1)))
          .onFalse(new InstantCommand(_intake::stopIntakeMotor));

      new JoystickButton(_buttonPanel, 6)
          .onTrue(new InstantCommand(_shooter::intake))
          .onFalse(new InstantCommand(_shooter::stopIntake));

      new JoystickButton(_buttonPanel, 8)
          .onTrue(new InstantCommand(_shooter::shoot))
          .onFalse(new InstantCommand(_shooter::stopShoot));

      new JoystickButton(_buttonPanel, 7)
          .onTrue(new IntakeCommand(-1, _intake))
          .onFalse(new IntakeCommand(0, _intake));

      new JoystickButton(_buttonPanel, 9)
          .onTrue(new IntakeCommand(1, _intake))
          .onFalse(new IntakeCommand(0, _intake));

      new JoystickButton(_buttonPanel, 10)
          .onTrue(new MoveIntakeCommand(-0.2, _intake))
          .onFalse(new MoveIntakeCommand(0, _intake));

      new JoystickButton(_buttonPanel, 11)
          .onTrue(new MoveIntakeCommand(0.2, _intake))
          .onFalse(new MoveIntakeCommand(0, _intake));
    }
  }

  public Command getTestPathCommand() {
    String pathName = SmartDashboard.getString("Path to Test", "Test Path");
    System.out.println(pathName);
    return NavUtilites.makePath(pathName, _drive);
  }

  /**
   * Sets up the named commands for the robot.
   * 
   * This method creates and registers several named commands for the robot. The
   * named commands include:
   * - TargetCommand: A command that does nothing and is used as a target for
   * other commands.
   * - ShootCommand: A command that shoots balls. It first calls the shoot method
   * of the shooter subsystem, then waits for 5 seconds, checks if the shooter is
   * at speed, and finally feeds balls into the shooter and sets the intake motor
   * to -0.75.
   * - PickUpCommand: A command that picks up notes. It first calls the intake
   * action command with the INTAKE action, then calls the intake action command
   * with the LOAD action.
   * - EjectCommand: A command that ejects notes. It calls the intake action
   * command with the EJECT action.
   * 
   * After creating and naming the commands, they are registered with the
   * NamedCommands class for use with the path planner.
   */
  

  public Command getCommand(CommandType type) {
    switch (type) {
      case kShoot:
        return CommandFactory.makeShootCommand(_shooter, _intake);
      case kPickup:
        return CommandFactory.makePickUpCommand(_intake);
      case kTarget:
        // TODO: fill in with target command
        return new InstantCommand();
      default:
        return new InstantCommand();

    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if(_buttonPanel.getRawButton(15)) { // Auto 4
      return makeCenterAutoCommand();
    } else if(_buttonPanel.getRawButton(17)) { // Auto 3
      return makeSideAutoCommand(false);
    } else if(_buttonPanel.getRawButton(16)) { // Auto 2
      return makeSideAutoCommand(true);
    } else { // Auto 1
      return new JustShootAuto(_shooter, _drive);
    }

    // An example command will be run in autonomous
  }

  public Command makeSideAutoCommand(boolean isLeft) {
    // set gyro to +/-60
    // shoot
    // drive off the speaker

    double angle = isLeft ? 60 : -60;

    _drive.setGyro(angle);

    return new RunCommand(_shooter::shoot, _shooter)
      .until(_shooter::atSpeed)
      .andThen(_shooter::feed)
      .andThen(new WaitCommand(0.15))
      .andThen(new RunCommand(() -> {
        _drive.drive(
          1, 
          0, 
          0, 
          true);
      }, _drive))
      .finallyDo(()-> {
        _shooter.stopShoot();
        _drive.drive(0, 0, 0, false);
      });
  }

  public Command makeCenterAutoCommand()
    {
      return new RunCommand(_shooter::shoot, _shooter)
      .until(_shooter::atSpeed)
      .andThen(_shooter::feed)
      .andThen(new WaitCommand(0.15))
      .andThen(new InstantCommand(() -> {
        _intake.setPosition(Constants.IntakeConstants.kDeployedPos);
      }, _intake))
      .andThen(new WaitCommand(0.25))
      .andThen(_shooter::stopShoot)
      .andThen(_shooter::stopFeed)
      .andThen(new WaitCommand(0.15))
      .andThen(new ParallelRaceGroup(
          new RunCommand(() -> _intake.setIntakeMotor(1.0), _intake)
              .until(_intake::hasGamePiece),

          new RunCommand(() -> _drive.drive(
              1,
              0,
              0,
              true),
              _drive)
              .withTimeout(2.5)))
      .withTimeout(2.5)
      .andThen(() -> {_drive.drive(0, 0, 0, fieldCentric);})
      //.andThen(_intake::stopIntakeMotor)
      .andThen(() -> _intake.setPosition(Constants.IntakeConstants.kRetractedPos))
      .andThen(new ParallelCommandGroup(
          new RunCommand(() -> _drive.drive(
              -1.5,
              0,
              0,
              true),
              _drive)
              .withTimeout(1.0)))
      .andThen(_drive::stopMotors)
      .andThen(new RunCommand(_shooter::shoot, _shooter)
          .until(() -> _shooter.atSpeed() && _intake.inFirePosition()))
      .finallyDo(() -> {
        _shooter.stopFeed();
        _shooter.stopIntake();
        _intake.stopIntakeMotor();
      }); 
    }
    public Command makeShootCommand(){
      return new RunCommand(() -> {
          _shooter.shoot();
        }, _shooter)
        .until(_shooter::atSpeed)
        .withTimeout(2)
        .andThen(new RunCommand(() -> {
          _shooter.feed();
          _intake.setIntakeMotor(-0.75);
        }, _shooter, _intake))
        .finallyDo(() -> {
          _shooter.stopShoot();
          _intake.stopIntakeMotor();
        }); 
    }
    public Command make3NoteAutoCommand(StartingPosition startingPosition)
    {

      String secondNotePath = ""; 
      switch (startingPosition) {
        case kCenter:
          secondNotePath = "center far note"; 
        break; 
        case kLeft:
          secondNotePath = "left far note"; 
        break; 
        case kRight:
          secondNotePath = "right far note"; 
        break; 
      }
      return makeShootCommand().andThen(
        new ParallelRaceGroup(
          new RunCommand(() -> _intake.setIntakeMotor(1.0), _intake)
              .until(_intake::hasGamePiece),

          new RunCommand(() -> _drive.drive(
              1,
              0,
              0,
              true),
              _drive)
              .withTimeout(2.5))
        )
        .andThen(
          makeShootCommand()
        )
        .andThen(
          new ParallelCommandGroup(
            new RunCommand(() -> _intake.setIntakeMotor(1.0), _intake)
              .until(_intake::hasGamePiece),
            NavUtilites.makePath(secondNotePath, _drive) 
          )
        );
      
    }
}
