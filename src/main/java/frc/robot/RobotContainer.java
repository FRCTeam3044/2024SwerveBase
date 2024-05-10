// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.commands.AmpShooterCommand;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorSetAngleForAmpCommand;
import frc.robot.commands.ElevatorSetAngleForIntakeCommand;
import frc.robot.commands.ManualLobCommand;
import frc.robot.commands.ManualShooterCommand;
import frc.robot.commands.StateMachineCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeReverse;
import frc.robot.commands.TransitCommands.TransitCommand;
import frc.robot.commands.auto.AutoCommandFactory;
import frc.robot.commands.drive.DriveAndTrackPointCommand;
import frc.robot.commands.drive.ManualDriveCommand;
import frc.robot.commands.drive.XModeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.sim.SimStateMachine;
import frc.robot.tropath.commands.PathingCommandGenerator;
import frc.robot.tropath.robotprofile.RobotProfile;
import frc.robot.utils.AutoAiming;
import me.nabdev.pathfinding.autos.AutoParser;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

import java.io.FileNotFoundException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachineResetCommand;
import frc.robot.subsystems.TransitSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    public final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    public static final NoteDetection m_noteDetection = new NoteDetection();
    public static AutoAiming m_autoAiming;

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
    public static final StateMachine stateMachine;
    public final StateMachineCommand stateMachineCommand;

    public final PathingCommandGenerator pathingCommandGenerator;
    // public static boolean isRed = true;

    static {
        if (RobotBase.isSimulation()) {
            stateMachine = new SimStateMachine(shooter, elevator, transit, intake, m_noteDetection, m_robotDrive);
        } else {
            stateMachine = new StateMachine(shooter, elevator, transit, intake, m_noteDetection, m_robotDrive);
        }
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        stateMachineCommand = new StateMachineCommand(stateMachine);
        pathingCommandGenerator = new PathingCommandGenerator(
                new RobotProfile(PathfindingConstants.kMaxSpeedMetersPerSecond.get(),
                        PathfindingConstants.kMaxAccelerationMetersPerSecondSquared.get(),
                        PathfindingConstants.kMaxAngularSpeedRadiansPerSecond.get(),
                        PathfindingConstants.kMaxAngularAccelerationRadiansPerSecondSquared.get(),
                        DriveConstants.kRobotSize, DriveConstants.kRobotSize),
                m_robotDrive::getPose,
                m_robotDrive::driveSpeed,
                m_robotDrive,
                Field.CRESCENDO_2024)
                .withPhysicsAlgorithmType(true)
                .withAllianceFlipping(false);
        configureBindings();
        try {
            m_autoAiming = new AutoAiming(true);
        } catch (FileNotFoundException e) {
            DriverStation.reportError("Unable to load auto aiming data, check that it is in the right path", false);
            throw new RuntimeException(e);
        }

        m_robotDrive.setDefaultCommand(new ManualDriveCommand(this, m_robotDrive, m_driverController));
        climber.setDefaultCommand(new ClimberCommand(climber, m_operatorController.getHID()));
        // elevator.setDefaultCommand(new ElevatorTestCommand(elevator,
        // m_operatorController.getHID()));
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
        // Driver 1
        // m_driverController.rightTrigger().whileTrue(stateMachineCommand.onlyIf(() ->
        // !DriverStation.isTest()));
        Command autoAimAndAlignCommand = Commands.parallel(new AutoAimCommand(elevator, m_robotDrive),
                new DriveAndTrackPointCommand(m_robotDrive, m_driverController, true));
        m_driverController.leftTrigger().whileTrue(autoAimAndAlignCommand
                .onlyIf(() -> (!DriverStation.isTest() && !m_operatorController.getHID().getAButton())));
        // When the menu button is pressed*
        m_driverController.start()
                .onTrue(new StateMachineResetCommand(stateMachine).onlyIf(() -> !DriverStation.isTest()));
        m_driverController.x().whileTrue(new XModeCommand(m_robotDrive).onlyIf(() -> !DriverStation.isTest()));
        // Driver 2
        // Command manualIntakeCommand = Commands.parallel(new IntakeCommand(intake),
        // new TransitCommand(transit));
        m_operatorController.x().whileTrue((new IntakeCommand(intake)).onlyIf(() -> !DriverStation.isTest()));
        m_operatorController.y().whileTrue((new TransitCommand(transit).alongWith(new IntakeCommand(intake)))
                .onlyIf(() -> !DriverStation.isTest()));
        m_operatorController.leftTrigger()
                .whileTrue(new ManualShooterCommand(shooter, transit).onlyIf(() -> !DriverStation.isTest()));
        m_operatorController.leftBumper()
                .whileTrue(new ElevatorSetAngleForAmpCommand(elevator).onlyIf(() -> !DriverStation.isTest()));
        m_operatorController.rightBumper()
                .whileTrue((new AmpShooterCommand(shooter, transit).onlyIf(() -> !DriverStation.isTest())));
        m_operatorController.rightTrigger()
                .whileTrue(new ManualLobCommand(shooter, transit).onlyIf(() -> !DriverStation.isTest()));
        m_operatorController.a()
                .whileTrue(new ElevatorSetAngleForIntakeCommand(elevator).onlyIf(() -> !DriverStation.isTest()));
        m_operatorController.povDown().whileTrue((new IntakeReverse(intake)).onlyIf(() -> !DriverStation.isTest()));
        Command pickupNote = stateMachine.getPickupNoteCommand().onlyIf(() -> m_noteDetection.hasNote);
        // pickupNote.addRequirements(m_robotDrive);
        m_operatorController.b().whileTrue(pickupNote);

        // m_operatorController.b()
        // .whileTrue(new ElevatorSetAngleForSubwooferCommand(elevator).onlyIf(() ->
        // !DriverStation.isTest()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(String autoName) {
        // ArrayList<Pose2d> waypoints = new ArrayList<Pose2d>();
        // waypoints.add(new Pose2d(0, 1, new Rotation2d()));
        // waypoints.add(new Pose2d(1, 1, new Rotation2d()));
        // waypoints.add(new Pose2d(1, 0, new Rotation2d()));
        // waypoints.add(new Pose2d(0, 0, new Rotation2d()));
        // return new GoToAndTrackPointCommand(new Pose2d(4, 3, new Rotation2d()),
        // m_robotDrive);
        try {
            AutoCommandFactory.registerCommands();
            Command auto = AutoParser.loadAuto(autoName);

            return auto;
        } catch (FileNotFoundException e) {
            System.out.println("Couldn't find file");
            return null;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

}