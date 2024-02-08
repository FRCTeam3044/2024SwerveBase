// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.GoToAndTrackPoint;
import frc.robot.commands.GoToPoints;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.TargetRotationController;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.pathfinding.autos.AutoParser;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.function.Function;

import org.json.JSONObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final TargetRotationController m_targetRotController;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_targetRotController = new TargetRotationController(0, 0);
    new ConfigurableParameter<Double>(1.0, "Target X", m_targetRotController::setTargetX);
    new ConfigurableParameter<Double>(0.0, "Target Y", m_targetRotController::setTargetY);

    if (RobotBase.isReal()) {

      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () -> m_robotDrive.drive(
                  -MathUtil.applyDeadband(m_driverController.getLeftY(),
                      OIConstants.kDriveDeadband.get()),
                  -MathUtil.applyDeadband(m_driverController.getLeftX(),
                      OIConstants.kDriveDeadband.get()),
                  -MathUtil.applyDeadband(m_driverController.getRightX(),
                      OIConstants.kDriveDeadband.get()),

                  m_fieldRelative.get(), m_rateLimit.get()),
              m_robotDrive));
      // new RunCommand(() -> {
      // double rotOutput = -MathUtil.applyDeadband(m_driverController.getRightX(),
      // OIConstants.kDriveDeadband.get());
      // if (m_driverController.getRightTriggerAxis() > 0.5) {
      // rotOutput = m_targetRotController.calculate(m_robotDrive.getPose(),
      // m_robotDrive.getChassisSpeeds());
      // }
      // SmartDashboard.putNumber("Rotation controller output", rotOutput);
      // ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(
      // -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.15),
      // MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15), rotOutput);
      // targetChassisSpeeds =
      // ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds,
      // m_robotDrive.getPose().getRotation());
      // var targetModuleStates =
      // DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
      // m_robotDrive.setModuleStates(targetModuleStates);
      // }, m_robotDrive));
    } else {

      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () -> m_robotDrive.drive(
                  MathUtil.applyDeadband(m_driverController.getLeftY(),
                      OIConstants.kDriveDeadband.get()),
                  MathUtil.applyDeadband(m_driverController.getLeftX(),
                      OIConstants.kDriveDeadband.get()),
                  -MathUtil.applyDeadband(m_driverController.getRightX(),
                      OIConstants.kDriveDeadband.get()),

                  m_fieldRelative.get(), m_rateLimit.get()),
              m_robotDrive));

      // m_fieldRelative.get(), m_rateLimit.get()),
      // m_robotDrive));
      // new RunCommand(() -> {
      // double rotOutput = -MathUtil.applyDeadband(m_driverController.getRightX(),
      // OIConstants.kDriveDeadband.get());
      // if (m_driverController.getRightTriggerAxis() > 0.5) {
      // rotOutput = m_targetRotController.calculate(m_robotDrive.getPose(),
      // m_robotDrive.getChassisSpeeds());
      // }
      // SmartDashboard.putNumber("Rotation controller output", rotOutput);
      // ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds(
      // -MathUtil.applyDeadband(m_driverController.getLeftY(), 0.15),
      // MathUtil.applyDeadband(m_driverController.getLeftX(), 0.15), rotOutput);
      // targetChassisSpeeds =
      // ChassisSpeeds.fromFieldRelativeSpeeds(targetChassisSpeeds,
      // m_robotDrive.getPose().getRotation());
      // var targetModuleStates =
      // DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
      // m_robotDrive.setModuleStates(targetModuleStates);
      // }, m_robotDrive));

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
    ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
    waypoints.add(new Pose2d(0, 1, new Rotation2d()));
    waypoints.add(new Pose2d(1, 1, new Rotation2d()));
    waypoints.add(new Pose2d(1, 0, new Rotation2d()));
    waypoints.add(new Pose2d(0, 0, new Rotation2d()));
    // return new GoToAndTrackPoint(new Pose2d(4, 3, new Rotation2d()),
    // m_robotDrive);
    try {
      Function<JSONObject, Command> genWaitForNote = (JSONObject params) -> new WaitForNoteCommand();
      AutoParser.registerCommand("wait_for_note", genWaitForNote);
      Function<JSONObject, Command> genFollowPath = (JSONObject params) -> new GoToPoints(waypoints, m_robotDrive);
      AutoParser.registerCommand("follow_path", genFollowPath);
      Command auto = AutoParser
          .loadAuto(Filesystem.getDeployDirectory() + "/exampleAuto.json");
      return auto;
    } catch (FileNotFoundException e) {
      System.out.println("Couldnt find file");
      return null;
    }
  }

  public static class WaitForNoteCommand extends Command {
    private Timer mTimer = new Timer();

    @Override
    public void initialize() {
      System.out.println("Waiting For Note!");
      mTimer.reset();
      mTimer.start();
    }

    @Override
    public void execute() {
      System.out.println("Timer: " + mTimer.get());
    }

    @Override
    public boolean isFinished() {
      return mTimer.get() > 3;
    }
  }
}
