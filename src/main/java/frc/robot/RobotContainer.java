// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileNotFoundException;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommandFactory;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorManualControlCommand;
import frc.robot.commands.ManualShooterCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.TransitCommands.TransitCommand;
import frc.robot.commands.drive.DriveAndTrackPointCommand;
import frc.robot.commands.drive.GoToNoteCommand;
import frc.robot.commands.drive.ManualDriveCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import me.nabdev.pathfinding.autos.AutoParser;

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
  public static PowerDistribution m_powerDistroHub = new PowerDistribution();
  // The robot's subsystems and commands are defined here...
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  public static final NoteDetection m_noteDetection = new NoteDetection();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  public static final CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  public static final ClimberSubsystem climber = new ClimberSubsystem();
  public static final IntakeSubsystem intake = new IntakeSubsystem();
  public static final TransitSubsystem transit = new TransitSubsystem();
  public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
  public static final ShooterSubsystem shooter = new ShooterSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(new ManualDriveCommand(m_robotDrive, m_driverController));
    intake.setDefaultCommand(new IntakeCommand(intake, m_driverController.getHID()));
    climber.setDefaultCommand(new ClimberCommand(climber, m_driverController.getHID()));
    transit.setDefaultCommand(new TransitCommand(transit, m_driverController.getHID()));
    elevator.setDefaultCommand(new ElevatorManualControlCommand(elevator, m_driverController.getHID()));
    shooter.setDefaultCommand(new ManualShooterCommand(shooter, m_driverController.getHID()));

    SmartDashboard.putData("SystemStatus/AllSystemsCheck", allSystemsCheckCommand());
    SmartDashboard.putData("PDH", m_powerDistroHub);
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
    m_driverController.rightTrigger().whileTrue(new DriveAndTrackPointCommand(m_robotDrive, m_driverController));
    m_driverController.leftTrigger().whileTrue(new GoToNoteCommand(m_robotDrive, m_noteDetection));

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
    // return new GoToAndTrackPointCommand(new Pose2d(4, 3, new Rotation2d()),
    // m_robotDrive);
    try {
      AutoCommandFactory.registerCommands();
      Command auto = AutoParser.loadAuto("exampleAuto.json");
      return auto;
    } catch (FileNotFoundException e) {
      System.out.println("Couldn't find file");
      return null;
    }
  }

public static Command allSystemsCheckCommand() {
  if(m_robotDrive.getCurrentCommand() != null) {
    m_robotDrive.getCurrentCommand().cancel();
  }
    return Commands.sequence(
      m_robotDrive.getSystemCheckCommand(),
      transit.getSystemCheckCommand(),
        Commands.runOnce(
            () -> {
              if (allSystemsOK()) {
                // set leds to something
              } else {
                // check failed
                // set leds to something
              }
            }),
        Commands.waitSeconds(2.0)
        .andThen(() -> {
        })
    );
  }

  public static boolean allSystemsOK() {
    return transit.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK;
  }
}