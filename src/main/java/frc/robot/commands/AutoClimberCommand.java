// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.ClimberSubsystem;

public class AutoClimberCommand extends CommandBase {
    private final double k_P = 0;
    private final ClimberSubsystem m_climberSubsystem;
    private AHRS navX;

    private boolean needToEnd = false;

    private Button override;

    private double targetTheta = 0;

    public AutoClimberCommand(ClimberSubsystem subsystem, AHRS navX, Button override) {
        this.m_climberSubsystem = subsystem;
        this.override = override;

        // X points down, Y points intake-side
        this.navX = navX;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_climberSubsystem.climbing = true;
        m_climberSubsystem.running_automatic = true;
    }

    @Override
    public void execute() {
        double worldTheta = getWorldTheta();
        if (override.getAsBoolean()) {
            needToEnd = true;
        }
        m_climberSubsystem.setRotationSpeed(k_P * (targetTheta - worldTheta));
    }

    @Override
    public void end(boolean interrupted) {
        m_climberSubsystem.climbing = false;
        m_climberSubsystem.running_automatic = false;
    }

    @Override
    public boolean isFinished() {
        return needToEnd;
    }

    private double getWorldTheta() {
        double x_relative = navX.getRawAccelY();
        double y_relative = -navX.getRawAccelX();
        double theta = Math.atan2(y_relative, x_relative) + (Math.PI / 2.0);
        while (theta >= 2 * Math.PI) {
            theta -= 2 * Math.PI;
        }
        while (theta < 0) {
            theta += 2 * Math.PI;
        }
        System.out.println("Robot->World: " + (0.0 - theta));

        return computeBarAngle(getPistonLength(m_climberSubsystem)) - theta;
    }

    private double getPistonLength(ClimberSubsystem climber) {
        double ticks = (climber.getLeftActuatorPosition() + climber.getRightActuatorPosition()) / 2.0;
        double displacement = ticks * AUTO_CLIMBER_REVS_PER_TICK * AUTO_CLIMBER_METERS_PER_REV;
        return displacement + AUTO_CLIMBER_ZERO_PISTON_LENGTH;
    }

    private double computeBarAngle(double pistonLength) {
        double a = AUTO_CLIMBER_L;
        double b = Math.sqrt(Math.pow(AUTO_CLIMBER_H, 2) + Math.pow(AUTO_CLIMBER_B, 2));
        double c = pistonLength;
        double cos_gamma = (Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2)) / (2.0 * a * b);
        double gamma = Math.acos(cos_gamma);
        double alpha = Math.atan(AUTO_CLIMBER_B / AUTO_CLIMBER_H);
        return (Math.PI / 2.0) - (gamma + alpha);
    }
}
