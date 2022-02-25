// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  private final static int SHOOTER_RPM = 3000;
  private final static double SHOOT_SPEED_TOLERANCE = 0.1;

  private final ShooterSubsystem m_shooterSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final Button m_shooterToggle;
  private final DoubleSupplier m_shooterTrim;

  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem subsystem, IntakeSubsystem intakeSubsystem, Button shooterToggle, DoubleSupplier shooterTrim) {
    this.m_shooterSubsystem = subsystem;
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_shooterToggle = shooterToggle;
    this.m_shooterTrim = shooterTrim;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setShooterSpeed(trimSpeed(SHOOTER_RPM));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_shooterSubsystem.getCurrentShooterSpeed() - SHOOTER_RPM) / SHOOTER_RPM <= SHOOT_SPEED_TOLERANCE) {
      m_shooterSubsystem.setIndexerSpeed(trimSpeed(1));
      m_intakeSubsystem.setIntakeSpeed(trimSpeed(1));
    } else {
      m_shooterSubsystem.setIndexerSpeed(0);
      m_intakeSubsystem.setIntakeSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setShooterSpeed(0);
    m_shooterSubsystem.setIndexerSpeed(0);
    m_intakeSubsystem.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double directionalizeSpeed(double speed) {
    speed = Math.abs(speed);
    return m_shooterToggle.getAsBoolean() ? -speed : speed;
  }

  private double trimSpeed(double speed) {
    return directionalizeSpeed(speed) * m_shooterTrim.getAsDouble();
  }
}
