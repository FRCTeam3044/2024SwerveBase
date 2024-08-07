// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.StateMachineCommand;
import frc.robot.commands.TransitCommands.TransitCommand;
import frc.robot.commands.auto.AutoCommandFactory;
import frc.robot.commands.drive.DriveAndTrackPointCommand;
import frc.robot.commands.drive.ManualDriveCommand;
import frc.robot.commands.drive.XModeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.sim.SimStateMachine;
import frc.robot.utils.AutoAiming;
import frc.robot.utils.ConditionalXboxController;
import me.nabdev.oxconfig.ConfigurableParameter;
import me.nabdev.pathfinding.autos.AutoParser;

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
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.test;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop;

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

    public static final ConditionalXboxController m_driverTeleController = new ConditionalXboxController(
            m_driverController, teleop());
    public static final ConditionalXboxController m_operatorTeleController = new ConditionalXboxController(
            m_operatorController,
            teleop());
    public static final ConditionalXboxController m_testController = new ConditionalXboxController(m_driverController,
            test());

    public static final ClimberSubsystem climber = new ClimberSubsystem();
    public static final IntakeSubsystem intake = new IntakeSubsystem();
    public static final TransitSubsystem transit = new TransitSubsystem();
    public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public static final ShooterSubsystem shooter = new ShooterSubsystem();
    public static final StateMachine stateMachine;
    public final StateMachineCommand stateMachineCommand;

    private static ConfigurableParameter<Double> kElevatorPIDControlTarget = new ConfigurableParameter<Double>(
            0.0, "Elevator Test PID Target");

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
        configureTeleopBindings();
        configureTestBindings();
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
    private void configureTeleopBindings() {
        // Driver 1
        Command autoAimAndAlignCommand = Commands.parallel(elevator.autoAim(m_robotDrive),
                new DriveAndTrackPointCommand(m_robotDrive, m_driverController, true));
        m_driverTeleController.leftTrigger()
                .whileTrue(autoAimAndAlignCommand.onlyIf(() -> (!m_operatorController.getHID().getAButton())));
        // When the menu button is pressed*
        m_driverTeleController.start().onTrue(new StateMachineResetCommand(stateMachine));
        m_driverTeleController.x().whileTrue(new XModeCommand(m_robotDrive));

        m_operatorTeleController.x().whileTrue(intake.run());
        m_operatorTeleController.y().whileTrue(new TransitCommand(transit).alongWith(intake.run()));
        m_operatorTeleController.leftTrigger().whileTrue(shooter.speaker());
        m_operatorTeleController.leftBumper().whileTrue(elevator.amp());
        m_operatorTeleController.rightBumper().whileTrue(shooter.amp());
        m_operatorTeleController.rightTrigger().whileTrue(shooter.lob());
        m_operatorTeleController.a().whileTrue(elevator.intake());
        m_operatorTeleController.povDown().whileTrue(intake.run(true));
        m_operatorTeleController.b()
                .whileTrue(stateMachine.getPickupNoteCommand().onlyIf(() -> m_noteDetection.hasNote));
    }

    private void configureTestBindings() {
        m_testController.b().whileTrue(intake.run());
        m_testController.y().whileTrue(elevator.toAngle(kElevatorPIDControlTarget::get));
        m_testController.a().whileTrue(elevator.test(() -> -m_testController.controller.getRightY()));
        m_testController.povDown().whileTrue(shooter.slow());
        m_testController.povUp().whileTrue(shooter.shoot());
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