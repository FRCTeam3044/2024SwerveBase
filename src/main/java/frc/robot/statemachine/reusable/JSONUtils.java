package frc.robot.statemachine.reusable;

import org.json.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class JSONUtils {
    public static Pose2d getPose2d(JSONObject obj) {
        if (!obj.has("x") || !obj.has("y"))
            throw new IllegalArgumentException("JSONObject does not contain x and y keys");

        return new Pose2d(obj.getDouble("x"), obj.getDouble("y"),
                Rotation2d.fromDegrees(obj.has("rot") ? obj.getDouble("rot") : 0));
    }

    public static Translation2d getTranslation2d(JSONObject obj) {
        if (!obj.has("x") || !obj.has("y"))
            throw new IllegalArgumentException("JSONObject does not contain x and y keys");

        return new Translation2d(obj.getDouble("x"), obj.getDouble("y"));
    }
}
