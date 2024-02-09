// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.GoToPointSuppliedRotCommand;

public final class AutoCommandFactory {
  private AutoCommandFactory() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static GoToPointSuppliedRotCommand goToPoint(JSONObject parameters) {
    Pose2d target = new Pose2d(parameters.getDouble("targetX"), parameters.getDouble("targetY"), new Rotation2d());
    return new GoToPointSuppliedRotCommand(target, RobotContainer.m_robotDrive, new Rotation2d());
  }
}
