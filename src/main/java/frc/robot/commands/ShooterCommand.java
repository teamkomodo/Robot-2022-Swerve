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
  private final static double SHOOTER_SPEED = 3000;
  private final static double INJECTION_TOLERANCE = 0.1;

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
    m_shooterSubsystem.setShooterSpeed(correctSpeed(SHOOTER_SPEED + 200));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.setShooterSpeed(correctSpeed(SHOOTER_SPEED + 200));

    System.out.println("FLYWHEEL SPEED >> " + m_shooterSubsystem.getCurrentShooterSpeed());
    if(Math.abs(m_shooterSubsystem.getCurrentShooterSpeed() - correctSpeed(SHOOTER_SPEED)) / correctSpeed(SHOOTER_SPEED) <= INJECTION_TOLERANCE) {
      m_shooterSubsystem.setIndexerSpeed(correctSpeed(1));
      m_intakeSubsystem.setIntakeSpeed(correctSpeed(1));
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

  private double directionSpeed(double speed) {
    return m_shooterToggle.getAsBoolean() ? -speed : speed;
  }

  private double trimSpeed(double speed) {
    return speed * m_shooterTrim.getAsDouble();
  }

  private double correctSpeed(double speed) {
    return trimSpeed(directionSpeed(speed));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
