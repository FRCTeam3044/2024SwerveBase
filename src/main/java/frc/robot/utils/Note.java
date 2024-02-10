package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;

public class Note {
    public ArrayList<Pose2d> history = new ArrayList<Pose2d>();

    public Pose2d currentPose;

    public int lostFrames;

}