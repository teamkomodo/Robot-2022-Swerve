// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  private final GenericHID leftJoystick = new GenericHID(0);
  private final GenericHID rightJoystick = new GenericHID(1);

  private final GenericHID OCButtonController = new GenericHID(2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        m_climberSubsystem,
        () -> -modifyAxis(rightJoystick.getRawAxis(0)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(rightJoystick.getRawAxis(1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(leftJoystick.getRawAxis(0)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Button gyroZeroButton = new Button(() -> leftJoystick.getRawButton(0));

    Button intakeButton = new Button(() -> OCButtonController.getRawButton(3));
    Button intakeToggle = new Button(() -> OCButtonController.getRawButton(0));
    DoubleSupplier intakeTrimSupplier = () -> (OCButtonController.getRawAxis(0) + 1)/2;

    Button shooterButton = new Button(() -> OCButtonController.getRawButton(4));
    Button shooterToggle = new Button(() -> OCButtonController.getRawButton(1));
    DoubleSupplier shooterTrimSupplier = () -> (OCButtonController.getRawAxis(1) + 1)/2;
    
    Button climberButton = new Button(() -> OCButtonController.getRawButton(5));
    Button climberToggle = new Button(() -> OCButtonController.getRawButton(2));
    DoubleSupplier climberRotationSupplier = () -> leftJoystick.getRawAxis(0);
    DoubleSupplier climberChainsawSuppler = () -> rightJoystick.getRawAxis(0);

    // Left button zeros the gyroscope
    gyroZeroButton.whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    intakeButton.whileHeld(new IntakeCommand(m_intakeSubsystem, intakeToggle, intakeTrimSupplier));
    shooterButton.whileHeld(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, shooterToggle, shooterTrimSupplier));
    climberButton.toggleWhenPressed(new ClimberCommand(m_climberSubsystem, climberToggle, climberRotationSupplier, climberChainsawSuppler));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
