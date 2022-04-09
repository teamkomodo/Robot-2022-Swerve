// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoCommand extends CommandBase {
  private static final double INTAKE_RUN_LENGTH = 1; // Time to run intake at start of auto (seconds)

  private static final double CHAINSAW_RUN_LENGTH = 0.5; // Time to run chainsaw at start of auto (seconds)

  private static final double SHOOTER_DELAY = 1; // Time after auto start to start flywheel (seconds)
  private static final double SHOOTER_RUN_LENGTH = 2.5; // Time to run shooter after flywheel start (seconds)
  private static final double SHOOTER_SPEED = 2500; // Target RPM for flywheel (RPM)

  private static final double DRIVEBACK_DELAY = 5; // Time after auto start to drive backwards (seconds)
  private static final double DRIVEBACK_TIME = 0.6; // Time to drive backwards (seconds)

  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ClimberSubsystem m_climberSubsystem;

  private final Timer autoTimer;

  /** Creates a new AutoComand. */
  public AutoCommand(DrivetrainSubsystem drivetrainSubsystem, IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem, ClimberSubsystem climberSubsystem) {
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_climberSubsystem = climberSubsystem;

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
    if (autoTimer.get() < INTAKE_RUN_LENGTH) {
      m_intakeSubsystem.setIntakeSpeed(1);
    } else {
      m_intakeSubsystem.setIntakeSpeed(0);
    }

    // Move chainsaw to correct position
    if (autoTimer.get() <= CHAINSAW_RUN_LENGTH) {
      m_climberSubsystem.setChainsawSpeed(1);
    } else {
      m_climberSubsystem.setChainsawSpeed(0);
    }

    // Start shooter spinup
    if (autoTimer.get() >= SHOOTER_DELAY && autoTimer.get() <= SHOOTER_DELAY + SHOOTER_RUN_LENGTH) {
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

    // Drive back
    if (autoTimer.get() >= DRIVEBACK_DELAY && autoTimer.get() <= DRIVEBACK_DELAY + DRIVEBACK_TIME) {
      m_drivetrainSubsystem.drive(new ChassisSpeeds(-5.0, 0.0, 0.0));
    } else {
      m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
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
    m_climberSubsystem.setChainsawSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
