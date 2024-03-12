// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import org.json.JSONObject;
import org.json.JSONArray;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.ElevatorSetAngleForAmpCommand;
import frc.robot.commands.ElevatorSetAngleForIntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.TransitCommands.TransitCommand;
import frc.robot.commands.drive.GoToAndTrackPointCommand;
import frc.robot.commands.drive.GoToNoteCommand;
import frc.robot.commands.drive.GoToPointSuppliedRotCommand;
import me.nabdev.pathfinding.autos.AutoParser;

public final class AutoCommandFactory {
    private static boolean hasRegisteredCommands = false;
    public static double fieldCenter = 8.275;

    private AutoCommandFactory() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static void registerCommands() {
        if (hasRegisteredCommands) {
            return;
        }
        hasRegisteredCommands = true;
        AutoParser.registerCommand("go_to_point", AutoCommandFactory::goToPointConstantRot);
        AutoParser.registerCommand("go_to_waypoints", AutoCommandFactory::goToPointsConstantRot);
        AutoParser.registerCommand("go_to_waypoints_and_track", AutoCommandFactory::goToPointsAndTrack);
        AutoParser.registerCommand("go_to_and_track_point", AutoCommandFactory::goToAndTrackPoint);
        AutoParser.registerCommand("wait_for_note", AutoCommandFactory::waitForNote);
        AutoParser.registerCommand("wait_for_note_in_area", AutoCommandFactory::waitForNoteInArea);
        AutoParser.registerCommand("require_note", AutoCommandFactory::requireNote);
        AutoParser.registerCommand("go_to_note", AutoCommandFactory::goToNote);
        AutoParser.registerCommand("go_to_note_in_area", AutoCommandFactory::goToNoteInArea);
        AutoParser.registerCommand("set_intake_angle", AutoCommandFactory::elevatorSetAngleForIntakeCommand);
        AutoParser.registerCommand("set_amp_angle", AutoCommandFactory::elevatorSetAngleForAmpCommand);
        AutoParser.registerCommand("intake", AutoCommandFactory::intakeCommand);
        AutoParser.registerCommand("transit", AutoCommandFactory::transitCommand);
        AutoParser.registerCommand("wait_for_limit_switch", AutoCommandFactory::waitForLimitSwitch);
        AutoParser.registerCommand("to_shooting_zone", AutoCommandFactory::getToShootingZone);
        AutoParser.registerCommand("auto_aim_shooter", AutoCommandFactory::autoAimShooter);
        AutoParser.registerCommand("spinup_shooter_if_in_range", AutoCommandFactory::spinupShooterIfInRange);
        AutoParser.registerCommand("shoot_if_ready", AutoCommandFactory::shootIfReady);
        AutoParser.registerBoolean("note_in_area", AutoCommandFactory::noteInArea);
        AutoParser.registerBoolean("has_note", AutoCommandFactory::hasNote);
        AutoParser.registerBoolean("is_state", AutoCommandFactory::isState);
        AutoParser.registerMacro("pickup_note", "PickupNote.json");
        AutoParser.registerMacro("score_note", "ScoreNoteIfHave.json");
        AutoParser.registerMacro("pickup_and_score", "PickupAndScoreNote.json");
    }

    public static Command shootIfReady(JSONObject parameters) {
        return new ShootIfReady(RobotContainer.transit);
    }

    public static Command spinupShooterIfInRange(JSONObject parameters) {
        return new SpinupShooterIfInRange(RobotContainer.shooter, RobotContainer.m_robotDrive);
    }

    public static Command autoAimShooter(JSONObject parameters) {
        return new AutoAimCommand(RobotContainer.elevator, RobotContainer.m_robotDrive);
    }

    public static Command getToShootingZone(JSONObject parameters) {
        return new GoToShootingZone(RobotContainer.m_robotDrive);
    }

    public static NoteInArea noteInArea(JSONObject parameters) {
        Pose2d target = getAllianceLocation(parameters.getDouble("regionX"), parameters.getDouble("regionY"));
        double targetRadius = parameters.getDouble("regionRadius");
        return new NoteInArea(RobotContainer.m_noteDetection, target, targetRadius, false);
    }

    public static IsState isState(JSONObject parameters) {
        return new IsState(RobotContainer.stateMachine, parameters.getString("state"));
    }

    public static HasNote hasNote(JSONObject parameters) {
        return new HasNote(RobotContainer.stateMachine);
    }

    public static WaitForLimitSwitchCommand waitForLimitSwitch(JSONObject parameters) {
        boolean waitForOpen = parameters.getBoolean("waitForOpen");
        double time = parameters.getDouble("time");
        if (parameters.getString("subsystem") == "intake") {
            return new WaitForLimitSwitchCommand(RobotContainer.intake, waitForOpen, time);
        } else if (parameters.getString("subsystem") == "transit") {
            return new WaitForLimitSwitchCommand(RobotContainer.transit, waitForOpen, time);
        } else {
            throw new IllegalArgumentException(
                    "Invalid subsystem in limit switch auto command: " + parameters.getString("subsystem"));
        }
    }

    public static TransitCommand transitCommand(JSONObject parameters) {
        return new TransitCommand(RobotContainer.transit);
    }

    public static GoToNoteCommand goToNote(JSONObject parameters) {
        return new GoToNoteCommand(RobotContainer.m_robotDrive, RobotContainer.m_noteDetection, true);
    }

    public static GoToNoteCommand goToNoteInArea(JSONObject parameters) {
        Pose2d target = getAllianceLocation(parameters.getDouble("regionX"), parameters.getDouble("regionY"));
        double targetRadius = parameters.getDouble("regionRadius");
        return new GoToNoteCommand(RobotContainer.m_robotDrive, RobotContainer.m_noteDetection, target, targetRadius,
                false);
    }

    public static WaitForNoteCommand waitForNote(JSONObject parameters) {
        return new WaitForNoteCommand(RobotContainer.m_noteDetection, false);
    }

    public static WaitForNoteCommand requireNote(JSONObject parameters) {
        return new WaitForNoteCommand(RobotContainer.m_noteDetection, true);
    }

    public static WaitForNoteCommand waitForNoteInArea(JSONObject parameters) {
        Pose2d target = getAllianceLocation(parameters.getDouble("regionX"), parameters.getDouble("regionY"));
        double targetRadius = parameters.getDouble("regionRadius");
        return new WaitForNoteCommand(RobotContainer.m_noteDetection, target, targetRadius, false);
    }

    public static GoToPointSuppliedRotCommand goToPointConstantRot(JSONObject parameters) {
        Pose2d target = getAllianceLocation(parameters.getDouble("targetX"), parameters.getDouble("targetY"));
        return new GoToPointSuppliedRotCommand(target, RobotContainer.m_robotDrive,
                Rotation2d.fromDegrees(parameters.getDouble("rotDegrees")));
    }

    public static GoToPointSuppliedRotCommand goToPointsConstantRot(JSONObject parameters) {
        JSONArray waypoints = parameters.getJSONArray("waypoints");
        ArrayList<Pose2d> targets = new ArrayList<Pose2d>();
        for (int i = 0; i < waypoints.length(); i++) {
            JSONObject waypoint = waypoints.getJSONObject(i);
            Pose2d waypointPose = getAllianceLocation(waypoint.getDouble("x"), waypoint.getDouble("y"));
            targets.add(waypointPose);
        }
        return new GoToPointSuppliedRotCommand(targets, RobotContainer.m_robotDrive,
                Rotation2d.fromDegrees(parameters.getDouble("rotDegrees")));
    }

    public static GoToAndTrackPointCommand goToPointsAndTrack(JSONObject parameters) {
        JSONArray waypoints = parameters.getJSONArray("waypoints");
        ArrayList<Pose2d> targets = new ArrayList<Pose2d>();
        for (int i = 0; i < waypoints.length(); i++) {
            JSONObject waypoint = waypoints.getJSONObject(i);
            Pose2d waypointPose = getAllianceLocation(waypoint.getDouble("x"), waypoint.getDouble("y"));
            targets.add(waypointPose);
        }
        Pose2d trackTarget = getAllianceLocation(parameters.getDouble("trackX"), parameters.getDouble("trackY"));
        boolean flipped = parameters.getBoolean("flipped");
        SmartDashboard.putNumberArray("track target", new double[] { trackTarget.getX(), trackTarget.getY() });
        return new GoToAndTrackPointCommand(targets, trackTarget, RobotContainer.m_robotDrive, flipped);
    }

    public static GoToAndTrackPointCommand goToAndTrackPoint(JSONObject parameters) {
        Pose2d target = getAllianceLocation(parameters.getDouble("targetX"), parameters.getDouble("targetY"));
        Pose2d trackTarget = getAllianceLocation(parameters.getDouble("trackX"), parameters.getDouble("trackY"));
        boolean flipped = parameters.getBoolean("flipped");
        return new GoToAndTrackPointCommand(target, trackTarget, RobotContainer.m_robotDrive, flipped);
    }

    public static ElevatorSetAngleForIntakeCommand elevatorSetAngleForIntakeCommand(JSONObject parameters) {
        return new ElevatorSetAngleForIntakeCommand(RobotContainer.elevator);
    }

    public static ElevatorSetAngleForAmpCommand elevatorSetAngleForAmpCommand(JSONObject parameters) {
        return new ElevatorSetAngleForAmpCommand(RobotContainer.elevator);
    }

    public static IntakeCommand intakeCommand(JSONObject parameters) {
        return new IntakeCommand(RobotContainer.intake);
    }

    private static Pose2d getAllianceLocation(double x, double y) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
                return new Pose2d(x, y, new Rotation2d());
            } else {
                return new Pose2d(2 * fieldCenter - x, y, new Rotation2d());
            }
        } else {
            return new Pose2d(x, y, new Rotation2d());
        }
    }
}
