// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_indexMotor;

  private final CANSparkMax m_leftShootMotor;
  private final CANSparkMax m_rightShootMotor;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    this.m_indexMotor = new CANSparkMax(15, MotorType.kBrushless);

    this.m_leftShootMotor = new CANSparkMax(16, MotorType.kBrushless);
    this.m_rightShootMotor = new CANSparkMax(17, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double speed) {
    speed /= 5676;
    m_leftShootMotor.set(speed);
    m_rightShootMotor.set(speed);
  }

  public void setIndexerSpeed(double speed) {
    m_indexMotor.set(speed);
  }

  public double getCurrentShooterSpeed() {
    return (m_leftShootMotor.getEncoder().getVelocity() + m_rightShootMotor.getEncoder().getVelocity()) / 2;
  }
}
