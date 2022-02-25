// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {
  private final ClimberSubsystem m_climberSubsystem;

  private final DoubleSupplier m_climberPosition;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(ClimberSubsystem subsystem, DoubleSupplier climberPosition) {
    this.m_climberSubsystem = subsystem;
    this.m_climberPosition = climberPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climberSubsystem.climbing = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.setRotationPosition(m_climberPosition.getAsDouble() * 100);
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
