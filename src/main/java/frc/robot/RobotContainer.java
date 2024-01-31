// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.commands.GoToPoints;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.TargetRotationController;
import me.nabdev.oxconfig.ConfigurableParameter;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  private final ConfigurableParameter<Boolean> m_fieldRelative = new ConfigurableParameter<Boolean>(true,
      "Field Relative");
  private final ConfigurableParameter<Boolean> m_rateLimit = new ConfigurableParameter<Boolean>(true, "Rate Limit");

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
                  MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband.get()),
                  MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband.get()),
                  MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband.get()),

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
    // ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
    // waypoints.add(new Pose2d(12, 6, new Rotation2d()));
    // waypoints.add(new Pose2d(13, 5, new Rotation2d()));
    // waypoints.add(new Pose2d(3, 3, new Rotation2d()));
    // return new GoToPoints(waypoints, m_robotDrive);
    TargetRotationController rotationController = new TargetRotationController(PathfindingConstants.kRotationFF.get(),
        PathfindingConstants.kRotationTimestep.get(), PathfindingConstants.kPathfindingThetaController, 0, 0);
    return new RunCommand(() -> {
      double rotOutput = rotationController.calculate(m_robotDrive.getPose(), m_robotDrive.getChassisSpeeds());
      ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(0, 0, rotOutput);
      var targetModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
      m_robotDrive.setModuleStates(targetModuleStates);
    }, m_robotDrive);
  }
}
