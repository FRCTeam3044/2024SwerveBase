// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StateMachineConstants;
import frc.robot.Constants.TransitConstants;
import frc.robot.utils.ControllerRumble;
import frc.robot.utils.USBLocator;
import frc.robot.commands.test.ClimberTestCommand;
import frc.robot.commands.test.DriveTestCommand;
import frc.robot.commands.test.ElevatorTestCommand;
import frc.robot.commands.test.IntakeTestCommand;
import frc.robot.commands.test.ShooterTestCommand;
import frc.robot.commands.test.TransitTestCommand;
import frc.robot.subsystems.LEDSubsystem;
import me.nabdev.oxconfig.OxConfig;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    public RobotContainer m_robotContainer;
    public LEDSubsystem m_led;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replaource
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter(USBLocator.getUSBPath() + "/logs"));
            Logger.addDataReceiver(new NT4Publisher());
        } else if (isSimulation()) {
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            return;
        }
        // Start AdvantageKit logger
        Logger.start();
        PathfindingConstants.initialize();
        DriveConstants.initialize();
        ElevatorConstants.initialize();
        ShooterConstants.initialize();
        StateMachineConstants.initialize();
        TransitConstants.initialize();
        OIConstants.initialize();
        IntakeConstants.initialize();
        ClimberConstants.initialize();
        m_robotContainer = new RobotContainer();
        OxConfig.initialize();
        m_led = new LEDSubsystem(LEDConstants.LEDPort, (LEDConstants.sideLEDLength * 2) + LEDConstants.topLEDLength, m_robotContainer);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        m_robotContainer.m_visionSubsystem.periodic();
        RobotContainer.m_noteDetection.periodic();
        ControllerRumble.updatePeriodic();
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    // private double[] lastClick = SmartDashboard.getNumberArray("ClickPosition",
    // new double[] { 0, 0 });

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // lastClick = SmartDashboard.getNumberArray("ClickPosition", new double[] { 0,
        // 0 });

        // RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble,
        // 1);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // RobotContainer.m_driverController.getHID().setRumble(RumbleType.kBothRumble,
        // 1);

        // double[] click = SmartDashboard.getNumberArray("ClickPosition", new double[]
        // { 0, 0 });
        // if (click[0] != lastClick[0] || click[1] != lastClick[1]) {
        // (new GoToPointDriverRotCommand(new Pose2d(click[0], click[1], new
        // Rotation2d()),
        // RobotContainer.m_robotDrive,
        // RobotContainer.m_driverController)).schedule();
        // lastClick = click;
        // }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        Command climberTestCommand = new ClimberTestCommand(RobotContainer.climber,
                RobotContainer.m_driverController.getHID());
        climberTestCommand.schedule();
        Command intakeTestCommand = new IntakeTestCommand(RobotContainer.intake,
                RobotContainer.m_driverController.getHID());
        intakeTestCommand.schedule();
        Command transitTestCommand = new TransitTestCommand(RobotContainer.transit,
                RobotContainer.m_driverController.getHID());
        transitTestCommand.schedule();
        Command driveTestCommand = new DriveTestCommand(m_robotContainer, RobotContainer.m_robotDrive,
                RobotContainer.m_driverController);
        driveTestCommand.schedule();
        Command elevatorTestCommand = new ElevatorTestCommand(RobotContainer.elevator,
                RobotContainer.m_driverController.getHID());
        elevatorTestCommand.schedule();
        Command shooterTestCommand = new ShooterTestCommand(RobotContainer.shooter,
                RobotContainer.m_driverController.getHID());
        shooterTestCommand.schedule();

    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        if (RobotContainer.m_driverController.getHID().getAButton()) {
            RobotContainer.elevator.calibrationModeEnabled = true;
        } else {
            RobotContainer.elevator.calibrationModeEnabled = false;
        }
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}