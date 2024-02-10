package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.Note;

public class TrackerSubsystem {
    ArrayList<Note> notes = new ArrayList<Note>();

    /*
     * get the new not positions
     */
    public ArrayList<Note> getNotePositions() {
        return notes;
    }

    /*
     * Set the new note positions
     */
    public void setNewNotePositions(ArrayList<Pose2d> detections) {
        double[][] distances = new double[detections.size()][notes.size()];
        for (int i = 0; i < detections.size(); i++) {
            for (int j = 0; j < notes.size(); j++) {
                Pose2d oldNote = notes.get(j).currentPose;
                Pose2d newNote = detections.get(i);
                distances[i][j] = findDistance(oldNote, newNote);
            }
        }
    }

    /*
     * Finds the distance between an old note and new note
     */
    public double findDistance(Pose2d oldNotePose, Pose2d newNotePose) {
        // Calculate the hypotenuse between the two notes then return the distance (in
        // meters)
        double xDistance = newNotePose.getX() - oldNotePose.getX();
        double yDistance = newNotePose.getY() - oldNotePose.getY();

        return Math.sqrt((xDistance * xDistance) + (yDistance * yDistance));
    }

    /*
     * Match the notes by saying this note is this note because it has the shortest
     * difference from it
     */
    public void mergeShortestDistances(double[][] distances) {

    }
}
