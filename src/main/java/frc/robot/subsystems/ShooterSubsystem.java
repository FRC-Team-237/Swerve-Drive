// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax lowMotor;
  private CANSparkMax highMotor;
  private CANSparkMax feederMotor;

  private SparkPIDController lowMotorPID;
  private SparkPIDController highMotorPID;

  private RelativeEncoder lowMotorEncoder;
  private RelativeEncoder highMotorEncoder;

  private double shooterRPM = 0.0;

  private double highShooterP = 0.0;
  private double highShooterI = 0.0;
  private double highShooterD = 0.0;
  private double highShooterFF = 0.0;
  private double lowShooterP = 0.0;
  private double lowShooterI = 0.0;
  private double lowShooterD = 0.0;
  private double lowShooterFF = 0.0;

  private double spitPower = 0.5;

  public ShooterSubsystem() {
    lowMotor = new CANSparkMax(Constants.Mechanism.kShooterLowMotorId, MotorType.kBrushless);
    highMotor = new CANSparkMax(Constants.Mechanism.kShooterHighMotorId, MotorType.kBrushless);
    feederMotor = new CANSparkMax(Constants.Mechanism.kShooterFeederMotorId, MotorType.kBrushless);

    lowMotor.setIdleMode(IdleMode.kCoast);
    highMotor.setIdleMode(IdleMode.kCoast);

    lowMotorEncoder = lowMotor.getEncoder();
    highMotorEncoder = highMotor.getEncoder();

    lowMotorPID = lowMotor.getPIDController();
    highMotorPID = highMotor.getPIDController();

    lowMotorPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    highMotorPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    SmartDashboard.putNumber("Shooter/highP", 0.0);
    SmartDashboard.putNumber("Shooter/highI", 0.0);
    SmartDashboard.putNumber("Shooter/highD", 0.0);
    SmartDashboard.putNumber("Shooter/highFF", 0.00018125);
    SmartDashboard.putNumber("Shooter/lowP", 0.0);
    SmartDashboard.putNumber("Shooter/lowI", 0.0);
    SmartDashboard.putNumber("Shooter/lowD", 0.0);
    SmartDashboard.putNumber("Shooter/lowFF", 0.00018125);
    SmartDashboard.putNumber("Shooter/RPM", Constants.Mechanism.kShooterMaxTargetRPM);
    SmartDashboard.putNumber("Shooter/SpitPower", 0.5);

    updateDashboardValues();
  }
  
  private void updateDashboardValues() {
    highShooterP = SmartDashboard.getNumber("Shooter/highP", 0.0);
    highShooterI = SmartDashboard.getNumber("Shooter/highI", 0.0);
    highShooterD = SmartDashboard.getNumber("Shooter/highD", 0.0);
    highShooterFF = SmartDashboard.getNumber("Shooter/highFF", 0.00018125);
    lowShooterP = SmartDashboard.getNumber("Shooter/lowP", 0.0);
    lowShooterI = SmartDashboard.getNumber("Shooter/lowI", 0.0);
    lowShooterD = SmartDashboard.getNumber("Shooter/lowD", 0.0);
    lowShooterFF = SmartDashboard.getNumber("Shooter/lowFF", 0.00018125);
    shooterRPM = SmartDashboard.getNumber("Shooter/RPM", Constants.Mechanism.kShooterMaxTargetRPM);

    spitPower = SmartDashboard.getNumber("Shooter/SpitPower", 0.5);

    lowMotorPID.setP(lowShooterP);
    lowMotorPID.setI(lowShooterI);
    lowMotorPID.setD(lowShooterD);
    lowMotorPID.setFF(lowShooterFF);
    highMotorPID.setP(highShooterP);
    highMotorPID.setI(highShooterI);
    highMotorPID.setD(highShooterD);
    highMotorPID.setFF(highShooterFF);
  }

  public void spit() {
    outputShooter(spitPower);
  }


  public void shoot() {
    outputShooter(1);
  }
  
  public void stopShoot() {
    outputShooter(0);
  }


  public void intake() {
    outputShooter(-Constants.Mechanism.kIntakeMultiplier);
    outputFeeder(-Constants.Mechanism.kShooterFeedMultiplier * Constants.Mechanism.kIntakeMultiplier);
  }

  public void stopIntake() {
    outputShooter(0);
    outputFeeder(0);
  }


  public void feed() {
    outputFeeder(0.5);
  }

  public void stopFeed() {
    outputFeeder(0);
  }


  public void outputShooter(double speed) {
    updateDashboardValues();
    if(Math.abs(speed) <= 0.01) {
      lowMotor.stopMotor();
      highMotor.stopMotor();
      return;
    }
    lowMotorPID.setReference(speed * shooterRPM, ControlType.kVelocity);
    highMotorPID.setReference(speed * shooterRPM, ControlType.kVelocity);
  }

  public void outputFeeder(double speed) {
    updateDashboardValues();
    feederMotor.set(-speed);
  }
  public boolean atSpeed(){
    if (highMotorEncoder.getVelocity()>shooterRPM-350) {
      return true;
    } else {
      return false;
    }
  } 
  @Override
  public void periodic() {
    log();
  }

  private void log() {
    SmartDashboard.putNumber("Shooter/Low Motor Speed", lowMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/High Motor Speed", highMotorEncoder.getVelocity());
  }
}
