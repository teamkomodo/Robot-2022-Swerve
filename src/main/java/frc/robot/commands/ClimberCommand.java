// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {
  private final ClimberSubsystem m_climberSubsystem;

  private final Button m_climberToggle;

  private final DoubleSupplier m_rotationSpeed;
  private final DoubleSupplier m_chainsawSpeed;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(ClimberSubsystem subsystem, Button climberToggle, DoubleSupplier rotationSpeed,
      DoubleSupplier chainsawSpeed) {
    this.m_climberSubsystem = subsystem;
    this.m_climberToggle = climberToggle;
    this.m_rotationSpeed = rotationSpeed;
    this.m_chainsawSpeed = chainsawSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climberSubsystem.climbing = m_climberToggle.getAsBoolean();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.climbing = m_climberToggle.getAsBoolean();

    if (m_climberSubsystem.climbing) {
      m_climberSubsystem.setRotationSpeed(m_rotationSpeed.getAsDouble());
      m_climberSubsystem.setChainsawSpeed(m_chainsawSpeed.getAsDouble());
    } else {
      m_climberSubsystem.setRotationSpeed(0);
      m_climberSubsystem.setChainsawSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.climbing = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
