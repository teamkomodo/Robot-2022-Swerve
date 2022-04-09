package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final double CHILL_MODE_POWER_PERCENTAGE = 0.3;

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final ClimberSubsystem m_climberSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private final Button m_chillModeToggle;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            ClimberSubsystem climberSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier,
            Button chillModeToggle) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_climberSubsystem = climberSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_chillModeToggle = chillModeToggle;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        if (!m_climberSubsystem.climbing) {
            if(!m_chillModeToggle.getAsBoolean()) {
                m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            m_rotationSupplier.getAsDouble(),
                            m_drivetrainSubsystem.getGyroscopeRotation()));
            } else {
                m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble() * CHILL_MODE_POWER_PERCENTAGE,
                            m_translationYSupplier.getAsDouble() * CHILL_MODE_POWER_PERCENTAGE,
                            m_rotationSupplier.getAsDouble() * CHILL_MODE_POWER_PERCENTAGE,
                            m_drivetrainSubsystem.getGyroscopeRotation()));
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
