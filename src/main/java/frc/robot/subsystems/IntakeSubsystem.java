// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_backIntakeMotor;
  private final CANSparkMax m_frontIntakeMotor;

  /** Creates a new Climber. */
  public IntakeSubsystem() {
    this.m_backIntakeMotor = new CANSparkMax(20, MotorType.kBrushless);
    this.m_frontIntakeMotor = new CANSparkMax(21, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    m_backIntakeMotor.set(speed);
    m_frontIntakeMotor.set(speed);
  }
}
