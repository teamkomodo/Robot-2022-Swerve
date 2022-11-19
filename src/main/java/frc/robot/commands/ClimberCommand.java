// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ClimberCommand extends CommandBase {
  private final ClimberSubsystem m_climberSubsystem;

  private final DoubleSupplier m_rotationSpeed;
  private final DoubleSupplier m_chainsawSpeed;

  private final Button m_rotationLimitSwitch;
  private final Button m_disableClimbLimitToggle;
  private final Button m_disableClimbOffsetLimitToggle;

  public static final double MAX_VEL_BASE_SLOW_MUL = 0.3;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(ClimberSubsystem subsystem, DoubleSupplier rotationSpeed,
      DoubleSupplier chainsawSpeed, Button rotationLimitSwitch, Button disableClimbLimitToggle,
      Button disableClimbOffsetLimitToggle) {
    this.m_climberSubsystem = subsystem;
    this.m_rotationSpeed = rotationSpeed;
    this.m_chainsawSpeed = chainsawSpeed;
    this.m_rotationLimitSwitch = rotationLimitSwitch;
    this.m_disableClimbLimitToggle = disableClimbLimitToggle;
    this.m_disableClimbOffsetLimitToggle = disableClimbOffsetLimitToggle;

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
    System.out.println("DIFF >> " + m_climberSubsystem.getRotationDiff());

    if (m_disableClimbLimitToggle.getAsBoolean()) {
      m_climberSubsystem.diableActuatorLimits();
    } else {
      m_climberSubsystem.enableActuatorLimits();
    }

    if (m_disableClimbOffsetLimitToggle.getAsBoolean()) {
      m_climberSubsystem.disableOffsetLimits();
    } else {
      m_climberSubsystem.enableOffsetLimits();
    }

    m_climberSubsystem.setRotationSpeed(m_rotationSpeed.getAsDouble() * (DrivetrainSubsystem.slowMode ? MAX_VEL_BASE_SLOW_MUL : 1.0));
    m_climberSubsystem.setChainsawSpeed(m_chainsawSpeed.getAsDouble() * (DrivetrainSubsystem.slowMode ? MAX_VEL_BASE_SLOW_MUL : 1.0));
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
