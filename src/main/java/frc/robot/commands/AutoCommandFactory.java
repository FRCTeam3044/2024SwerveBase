// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.JSONObject;
import org.json.JSONArray;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.GoToPointSuppliedRotCommand;
import me.nabdev.pathfinding.autos.AutoParser;

public final class AutoCommandFactory {
  private AutoCommandFactory() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static void registerCommands(){
    AutoParser.registerCommand("go_to_point", AutoCommandFactory::goToPointConstantRot);
    AutoParser.registerCommand("go_to_waypoints", AutoCommandFactory::goToPointsConstantRot);
  }

  public static GoToPointSuppliedRotCommand goToPointConstantRot(JSONObject parameters) {
    Pose2d target = new Pose2d(parameters.getDouble("targetX"), parameters.getDouble("targetY"), new Rotation2d());
    return new GoToPointSuppliedRotCommand(target, RobotContainer.m_robotDrive, Rotation2d.fromDegrees(parameters.getDouble("rotDegrees")));
  }

  public static GoToPointSuppliedRotCommand goToPointsConstantRot(JSONObject parameters){
    JSONArray waypoints = parameters.getJSONArray("waypoints");
    ArrayList<Pose2d> targets = new ArrayList<Pose2d>();
    for(int i = 0; i < waypoints.length(); i++){
      JSONObject waypoint = waypoints.getJSONObject(i);
      targets.add(new Pose2d(waypoint.getDouble("x"), waypoint.getDouble("y"), new Rotation2d()));
    }
    return new GoToPointSuppliedRotCommand(targets, RobotContainer.m_robotDrive, Rotation2d.fromDegrees(parameters.getDouble("rotDegrees")));
  }
}
