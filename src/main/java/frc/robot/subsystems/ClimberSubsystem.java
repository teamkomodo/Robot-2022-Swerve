// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX m_leftActuatorMotor;
  private final TalonFX m_rightActuatorMotor;

  private final CANSparkMax m_chainsawMotor;

  public boolean climbing;

  private PIDController rotationPIDController;
  private PIDController chainsawPIDController;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    this.m_leftActuatorMotor = new TalonFX(20);
    this.m_rightActuatorMotor = new TalonFX(21);
    this.m_chainsawMotor = new CANSparkMax(22, MotorType.kBrushless);

    this.climbing = false;

    this.rotationPIDController = new PIDController(0.5, 0, 0);
    this.chainsawPIDController = new PIDController(0.5, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private double getRotationDiff() {
    return (m_leftActuatorMotor.getSelectedSensorPosition() - m_rightActuatorMotor.getSelectedSensorPosition()) / 2048;
  }

  private boolean checkRotationAlignment() {
    if (Math.abs(getRotationDiff()) > 10) {
      return false;
    }

    return true;
  }

  private void setRotationSpeed(double speed) {
    if (!checkRotationAlignment()) {
      m_leftActuatorMotor.set(ControlMode.PercentOutput, 0);
      m_rightActuatorMotor.set(ControlMode.PercentOutput, 0);

      return;
    }

    m_leftActuatorMotor.set(ControlMode.PercentOutput, speed);
    m_rightActuatorMotor.set(ControlMode.PercentOutput, speed + getRotationDiff() / 5);
  }

  private double getRotationPosition() {
    return m_leftActuatorMotor.getSelectedSensorPosition() / 2048;
  }

  public void setRotationPosition(double position) {
    if (position < 0) {
      position = 0;
    } else if (position > 100) {
      position = 100;
    }

    setRotationSpeed(rotationPIDController.calculate(getRotationPosition(), position));
  }

  private void setChainsawSpeed(double speed) {
    m_chainsawMotor.set(speed);
  }

  private double getChainsawPosition() {
    return m_chainsawMotor.getEncoder().getPosition();
  }

  public void setChainsawPosition(double position) {
    if (position < 0) {
      position = 0;
    }

    setChainsawSpeed(chainsawPIDController.calculate(getChainsawPosition(), position));
  }
}
