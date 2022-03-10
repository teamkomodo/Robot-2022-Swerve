// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoComand extends CommandBase {
  private static final double SHOOTER_SPEED = 3000;
  private static final double DRIVEBACK_TIME = 1;

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  private final Timer autoTimer;

  /** Creates a new AutoComand. */
  public AutoComand(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;

    this.autoTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem, intakeSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoTimer.reset();
    autoTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run intake to drop out of shooter path
    if (autoTimer.get() < 1) {
      m_intakeSubsystem.setIntakeSpeed(1);
    } else {
      m_intakeSubsystem.setIntakeSpeed(0);
    }

    // Drive back to get in position for shooter
    if (autoTimer.get() <= DRIVEBACK_TIME) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 1, 0));
    } else {
      m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    // Shooter after driveback
    if (autoTimer.get() > DRIVEBACK_TIME) {
      m_shooterSubsystem.setShooterSpeed(SHOOTER_SPEED);
      if (Math.abs(m_shooterSubsystem.getCurrentShooterSpeed() - SHOOTER_SPEED) / SHOOTER_SPEED <= 0.1) { // Checks if
                                                                                                          // flywheel is
                                                                                                          // up to speed
        m_shooterSubsystem.setIndexerSpeed(1);
        m_intakeSubsystem.setIntakeSpeed(1);
      } else {
        m_shooterSubsystem.setIndexerSpeed(0);
        m_intakeSubsystem.setIntakeSpeed(0);
      }
    } else {
      m_intakeSubsystem.setIntakeSpeed(0);
      m_shooterSubsystem.setIndexerSpeed(0);
      m_shooterSubsystem.setShooterSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop all robot movement
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    m_intakeSubsystem.setIntakeSpeed(0);
    m_shooterSubsystem.setIndexerSpeed(0);
    m_shooterSubsystem.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
