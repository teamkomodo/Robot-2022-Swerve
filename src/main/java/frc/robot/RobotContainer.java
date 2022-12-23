// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.VisionCommand;
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

  // private final GenericHID leftJoystick = new GenericHID(0);
  // private final GenericHID rightJoystick = new GenericHID(1);

  // private final GenericHID OCButtonController = new GenericHID(2);

  private final XboxController OCXboxController = new XboxController(3);

  // private final DigitalInput rotationLimitSwitchInput = new DigitalInput(0);

  public static final Field2d field2d = new Field2d();

  // private static final boolean useXBOXDrive = true;

  // // Easiest to define this here so I don't have to pass through scopes with dependency injection (Gross Code)
  // Button chillModeToggle = new Button(() -> OCButtonController.getRawButton(2));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //     m_drivetrainSubsystem,
    //     m_climberSubsystem,
    //     () -> -modifyAxis((useXBOXDrive ? OCXboxController.getRightX() : 0) + rightJoystick.getRawAxis(1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis((useXBOXDrive ? OCXboxController.getRightY() : 0) + rightJoystick.getRawAxis(0)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //     () -> -modifyAxis((useXBOXDrive ? OCXboxController.getLeftX() : 0) + rightJoystick.getRawAxis(2)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
    //     chillModeToggle));

    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putData("Field", field2d);
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
    // Button gyroZeroButton = new Button(() -> rightJoystick.getRawButton(1));

    // Button intakeButton = new Button(() -> OCXboxController.getRightBumper());
    // Button intakeReverseButton = new Button(() -> OCXboxController.getLeftBumper());
    // DoubleSupplier intakeTrimSupplier = () -> (OCButtonController.getRawAxis(0) + 1) / 2;

    // Button shooterButton = new Button(() -> OCXboxController.getRightTriggerAxis() >= 0.5 ? true : false);
    // Button shooterReverseButton = new Button(() -> OCXboxController.getLeftTriggerAxis() >= 0.5 ? true : false);
    // DoubleSupplier shooterTrimSupplier = () -> (OCButtonController.getRawAxis(1) + 1) / 2;
    Button visionButton = new Button(() -> OCXboxController.getAButton());

    // Button climbEnableButton = new Button(() -> leftJoystick.getRawButton(1));
    // DoubleSupplier climberRotationSupplier = () -> -leftJoystick.getRawAxis(1);
    // DoubleSupplier climberChainsawSuppler = () -> rightJoystick.getRawAxis(1);
    // Button rotationLimitSwitch = new Button(() -> !rotationLimitSwitchInput.get());
    // Button disableClimbLimitToggle = new Button(() -> OCButtonController.getRawButton(3));
    // Button disableClimbOffsetLimitToggle = new Button(() -> OCButtonController.getRawButton(1));

    // Left button zeros the gyroscope
    // gyroZeroButton.whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    // intakeButton.whileHeld(new IntakeCommand(m_intakeSubsystem, false, intakeTrimSupplier));
    // intakeReverseButton.whileHeld(new IntakeCommand(m_intakeSubsystem, true, intakeTrimSupplier));

    // shooterButton
    //     .whileHeld(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, false, shooterTrimSupplier));
    // shooterReverseButton
    //     .whileHeld(new ShooterCommand(m_shooterSubsystem, m_intakeSubsystem, true, shooterTrimSupplier));
    visionButton.whileHeld(new VisionCommand());

    
    // Button composition so that climb isn't enabled while chill mode is enabled (Gross Code)
    // new Button(() -> climbEnableButton.and(new Button(() -> !chillModeToggle.get())).get()).toggleWhenPressed(
    //     new ClimberCommand(m_climberSubsystem, climberRotationSupplier, climberChainsawSuppler, rotationLimitSwitch,
    //         disableClimbLimitToggle, disableClimbOffsetLimitToggle));

    // rotationLimitSwitch.whenPressed(m_climberSubsystem::resetActuatorPosition);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoCommand(m_drivetrainSubsystem, m_intakeSubsystem, m_shooterSubsystem, m_climberSubsystem);
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
    value = Math.copySign(Math.pow(Math.abs(value), 2), value);

    return value;
  }

}
