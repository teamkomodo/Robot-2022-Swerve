// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX m_leftActuatorMotor;
  private final TalonFX m_rightActuatorMotor;

  private final CANSparkMax m_chainsawMotor;

  public boolean climbing;

  private double leftActuatorMotorEncoderOffset;
  private double rightActuatorMotorEncoderOffset;

  private boolean enableActuatorLimits;
  private boolean enableOffsetLimits;
  private boolean enableChainsawLimits;

  public boolean running_automatic = false;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    this.m_leftActuatorMotor = new TalonFX(13);
    this.m_rightActuatorMotor = new TalonFX(12);
    this.m_chainsawMotor = new CANSparkMax(18, MotorType.kBrushless);

    this.climbing = false;

    this.leftActuatorMotorEncoderOffset = 0;
    this.rightActuatorMotorEncoderOffset = 0;

    this.enableActuatorLimits = true;
    this.enableOffsetLimits = true;
    this.enableChainsawLimits = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getRotationDiff() {
    return (getLeftActuatorPosition() - getRightActuatorPosition()) / 2048;
  }

  public double getLeftActuatorPosition() {
    return m_leftActuatorMotor.getSelectedSensorPosition() - leftActuatorMotorEncoderOffset;
  }

  public double getRightActuatorPosition() {
    return m_rightActuatorMotor.getSelectedSensorPosition() - rightActuatorMotorEncoderOffset;
  }

  private boolean checkRotationAlignment() {
    if (Math.abs(getRotationDiff()) > 25) {
      return false;
    }

    return true;
  }

  private double getRotationPosition() {
    return getLeftActuatorPosition() / 2048;
  }

  public void setRotationSpeed(double speed) {
    System.out.println("Rotation >> " + getRotationPosition());
    if (!checkRotationAlignment() && enableOffsetLimits) {
      m_leftActuatorMotor.set(ControlMode.PercentOutput, 0);
      m_rightActuatorMotor.set(ControlMode.PercentOutput, 0);

      return;
    }
    if (getRotationPosition() > 253 && speed >= 0 && enableActuatorLimits) {
      m_leftActuatorMotor.set(ControlMode.PercentOutput, 0);
      m_rightActuatorMotor.set(ControlMode.PercentOutput, 0);

      return;
    }
    if (getRotationPosition() < 0 && speed <= 0 && enableActuatorLimits) {
      m_leftActuatorMotor.set(ControlMode.PercentOutput, 0);
      m_rightActuatorMotor.set(ControlMode.PercentOutput, 0);

      return;
    }
    m_leftActuatorMotor.set(ControlMode.PercentOutput, speed);
    m_rightActuatorMotor.set(ControlMode.PercentOutput, speed + getRotationDiff() / 5);
  }

  private double getChainsawPosition() {
    return m_chainsawMotor.getEncoder().getPosition();
  }

  public void setChainsawSpeed(double speed) {
    System.out.println("Chainsaw >> " + getChainsawPosition());
    if (getChainsawPosition() > 89 && speed >= 0 && enableChainsawLimits) {
      m_chainsawMotor.set(0);

      return;
    }
    if (getChainsawPosition() < -92 && speed <= 0 && enableChainsawLimits) {
      m_chainsawMotor.set(0);

      return;
    }

    m_chainsawMotor.set(speed);
  }

  public void resetActuatorPosition() {
    leftActuatorMotorEncoderOffset = m_leftActuatorMotor.getSelectedSensorPosition();
    rightActuatorMotorEncoderOffset = m_rightActuatorMotor.getSelectedSensorPosition();
  }

  public void diableActuatorLimits() {
    this.enableActuatorLimits = false;
  }

  public void enableActuatorLimits() {
    this.enableActuatorLimits = true;
  }

  public void disableOffsetLimits() {
    this.enableOffsetLimits = false;
  }

  public void enableOffsetLimits() {
    this.enableOffsetLimits = true;
  }

  public void enableChainsawLimits() {
    this.enableChainsawLimits = true;
  }

  public void disableChainsawLimits() {
    this.enableChainsawLimits = false;
  }
}
