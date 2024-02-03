// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import me.nabdev.oxconfig.ConfigurableParameter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public final CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  private final ConfigurableParameter<Boolean> m_fieldRelative = new ConfigurableParameter<Boolean>(true,
      "Field Relative");
  private final ConfigurableParameter<Boolean> m_rateLimit = new ConfigurableParameter<Boolean>(true, "Rate Limit");

  public final ClimberSubsystem climber = new ClimberSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    if (RobotBase.isReal()) {
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () -> m_robotDrive.drive(
                  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband.get()),
                  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband.get()),
                  -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband.get()),

                  m_fieldRelative.get(), m_rateLimit.get()),
              m_robotDrive));
    } else {
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () -> m_robotDrive.drive(
                  MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband.get()),
                  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband.get()),
                  -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband.get()),

                  m_fieldRelative.get(), m_rateLimit.get()),
              m_robotDrive));
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
