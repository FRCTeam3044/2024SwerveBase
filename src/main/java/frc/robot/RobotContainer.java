// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.oxconfig.sampleClasses.ConfigurablePIDController;
import me.nabdev.oxconfig.sampleClasses.ConfigurableProfiledPIDController;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.Edge;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
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
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  private final ConfigurableParameter<Boolean> m_fieldRelative = new ConfigurableParameter<Boolean>(true,
      "Field Relative");
  private final ConfigurableParameter<Boolean> m_rateLimit = new ConfigurableParameter<Boolean>(true, "Rate Limit");

  private final Pathfinder m_pathfinder;

  private final PIDController m_xController = new ConfigurablePIDController(1, 0, 0, "X Controller");
  private final PIDController m_yController = new ConfigurablePIDController(1, 0, 0, "Y Controller");
  private final ProfiledPIDController m_thetaController = new ConfigurableProfiledPIDController(10, 0, 0,
      new TrapezoidProfile.Constraints(6.28, 3.14), "Theta Controller");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_pathfinder = new PathfinderBuilder(Field.CHARGED_UP_2023).setInjectPoints(true).build();
    m_robotDrive.resetOdometry(new Pose2d(2, 2, new Rotation2d(0)));

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
    try {
      TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
          AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      Trajectory myPath = m_pathfinder.generateTrajectory(m_robotDrive.getPose(), new Pose2d(13, 4, new Rotation2d(0)),
          config);
      m_robotDrive.field.getObject("Path").setTrajectory(myPath);
      HolonomicDriveController controller = new HolonomicDriveController(m_xController, m_yController,
          m_thetaController);

      Timer timer = new Timer();
      timer.start();
      Supplier<Rotation2d> targetRotSupplier = () -> Rotation2d.fromDegrees(timer.get() * 180);
      return new FollowTrajectory(myPath, controller, targetRotSupplier, m_robotDrive, m_robotDrive);
    } catch (ImpossiblePathException e) {
      System.out.println("Impossible path");
      return null;
    }
  }
}
