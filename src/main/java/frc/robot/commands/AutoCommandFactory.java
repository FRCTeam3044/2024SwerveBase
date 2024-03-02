// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.JSONObject;
import org.json.JSONArray;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.GoToAndTrackPointCommand;
import frc.robot.commands.drive.GoToNoteCommand;
import frc.robot.commands.drive.GoToPointSuppliedRotCommand;
import frc.robot.commands.drive.WaitForNoteCommand;
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
        AutoParser.registerCommand("go_to_note", AutoCommandFactory::goToNote);
        AutoParser.registerCommand("go_to_note_in_area", AutoCommandFactory::goToNoteInArea);
        AutoParser.registerCommand("set_intake_angle", AutoCommandFactory::elevatorSetAngleForIntakeCommand);
        AutoParser.registerCommand("set_amp_angle", AutoCommandFactory::elevatorSetAngleForAmpCommand);
        AutoParser.registerMacro("pickup_note", "pickupNote.json");
    }

    public static GoToNoteCommand goToNote(JSONObject parameters) {
        return new GoToNoteCommand(RobotContainer.m_robotDrive, RobotContainer.m_noteDetection, true);
    }

    public static GoToNoteCommand goToNoteInArea(JSONObject parameters) {
        Pose2d target = getAllianceLocation(parameters.getDouble("regionX"), parameters.getDouble("regionY"));
        double targetRadius = parameters.getDouble("regionRadius");
        return new GoToNoteCommand(RobotContainer.m_robotDrive, RobotContainer.m_noteDetection, target, targetRadius,
                true);
    }

    public static WaitForNoteCommand waitForNote(JSONObject parameters) {
        return new WaitForNoteCommand(RobotContainer.m_noteDetection);
    }

    public static WaitForNoteCommand waitForNoteInArea(JSONObject parameters) {
        Pose2d target = getAllianceLocation(parameters.getDouble("regionX"), parameters.getDouble("regionY"));
        double targetRadius = parameters.getDouble("regionRadius");
        return new WaitForNoteCommand(RobotContainer.m_noteDetection, target, targetRadius);
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
            System.out.println(waypointPose);
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
            System.out.println(waypointPose);
        }
        Pose2d trackTarget = getAllianceLocation(parameters.getDouble("trackX"), parameters.getDouble("trackY"));
        SmartDashboard.putNumberArray("track target", new double[] { trackTarget.getX(), trackTarget.getY() });
        return new GoToAndTrackPointCommand(targets, trackTarget, RobotContainer.m_robotDrive);
    }

    public static GoToAndTrackPointCommand goToAndTrackPoint(JSONObject parameters) {
        Pose2d target = getAllianceLocation(parameters.getDouble("targetX"), parameters.getDouble("targetY"));
        Pose2d trackTarget = getAllianceLocation(parameters.getDouble("trackX"), parameters.getDouble("trackY"));
        return new GoToAndTrackPointCommand(target, trackTarget, RobotContainer.m_robotDrive);
    }

    public static ElevatorSetAngleForIntakeCommand elevatorSetAngleForIntakeCommand(JSONObject parameters) {
        return new ElevatorSetAngleForIntakeCommand(RobotContainer.elevator);
    }

    public static ElevatorSetAngleForAmpCommand elevatorSetAngleForAmpCommand(JSONObject parameters) {
        return new ElevatorSetAngleForAmpCommand(RobotContainer.elevator);
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
