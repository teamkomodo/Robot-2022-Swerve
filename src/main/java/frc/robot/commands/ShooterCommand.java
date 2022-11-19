// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
  private final static double SHOOTER_SPEED = 3000;
  private final static double INJECTION_TOLERANCE = 0.1;

  private final static double MAX_VEL_BASE_SLOW_MUL = 0.5;

  private final ShooterSubsystem m_shooterSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;

  
  private final boolean m_reverseToggle;
  private final DoubleSupplier m_shooterTrim;

  /** Creates a new ShooterCommand. */
  public ShooterCommand(ShooterSubsystem subsystem, IntakeSubsystem intakeSubsystem, boolean reverseToggle,
      DoubleSupplier shooterTrim) {
    this.m_shooterSubsystem = subsystem;
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_reverseToggle = reverseToggle;
    this.m_shooterTrim = shooterTrim;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.setShooterSpeed(correctSpeed(SHOOTER_SPEED));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("FLYWHEEL SPEED >> " + m_shooterSubsystem.getCurrentShooterSpeed());
    if(m_reverseToggle) {
      m_shooterSubsystem.setShooterSpeed(-0.2);
      m_shooterSubsystem.setIndexerSpeed(-1);
      m_intakeSubsystem.setIntakeSpeed(-1);
      return;
    }

    m_shooterSubsystem.setShooterSpeed(correctSpeed(SHOOTER_SPEED) * (DrivetrainSubsystem.slowMode ? MAX_VEL_BASE_SLOW_MUL : 1.0));
    if (Math.abs(m_shooterSubsystem.getCurrentShooterSpeed() - correctSpeed(SHOOTER_SPEED))
        / correctSpeed(SHOOTER_SPEED) <= INJECTION_TOLERANCE) {
      m_shooterSubsystem.setIndexerSpeed(correctSpeed(0.2));
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

  private double trimSpeed(double speed) {
    return speed * m_shooterTrim.getAsDouble();
  }

  private double correctSpeed(double speed) {
    return trimSpeed(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
