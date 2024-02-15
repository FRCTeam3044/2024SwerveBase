// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.commands.drive.GoToPointDriverRotCommand;
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
public class Robot extends TimedRobot {
  private static Robot instance;
  private Command m_autonomousCommand;
  public RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    instance = this;
    addPeriodic(() -> SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime()), 1.0);
    PathfindingConstants.initialize();

    Logger.addDataReceiver(new NT4Publisher());

    // Start AdvantageKit logger
    Logger.start();
    m_robotContainer = new RobotContainer();
    OxConfig.initialize();
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
    double startTime = Timer.getFPGATimestamp();
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
    SmartDashboard.putData(CommandScheduler.getInstance());

    SmartDashboard.putNumber("RIOInputVoltage", RobotController.getInputVoltage());
    SmartDashboard.putNumber("RIOCANUtil", RobotController.getCANStatus().percentBusUtilization * 100);
    SmartDashboard.putNumber("RobotPeriodicMS", (Timer.getFPGATimestamp() - startTime) * 1000);

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

  private double[] lastClick = SmartDashboard.getNumberArray("ClickPosition", new double[] { 0, 0 });

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    lastClick = SmartDashboard.getNumberArray("ClickPosition", new double[] { 0, 0 });
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double[] click = SmartDashboard.getNumberArray("ClickPosition", new double[] { 0, 0 });
    if (click[0] != lastClick[0] || click[1] != lastClick[1]) {
      (new GoToPointDriverRotCommand(new Pose2d(click[0], click[1], new Rotation2d()), RobotContainer.m_robotDrive,
          RobotContainer.m_driverController)).schedule();
      lastClick = click;
    }
    m_robotContainer.climber.leftArm(0);
    m_robotContainer.climber.rightArm(0);

    boolean isBButtonPressed = RobotContainer.m_operatorController.getHID().getBButtonPressed();

    m_robotContainer.intake.consumeIntakeInput(isBButtonPressed);

    RobotContainer.m_operatorController.getHID().getLeftY();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  public static void addPeriodicCallback(Runnable callback, double periodSeconds) {
    // Don't add the callback if the instance is null. This is pretty much just to make unit tests
    // work
    if (instance == null) {
      return;
    }

    instance.addPeriodic(callback, periodSeconds);
  }
}
