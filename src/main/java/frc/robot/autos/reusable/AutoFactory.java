package frc.robot.autos.reusable;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.AutoSegments;

public class AutoFactory {
    /**
     * Load an auto from a json file by converting each block to an AutoSegment
     * 
     * @param filePath The path to the file
     * @return The list of event loops
     */
    public static Command loadAuto(String filePath) {
        // Loads the file as a json array

        // For each element converts it to an AutoSegment (a command)

        // Returns a sequence of all the AutoSegments
        return null;
    }

    public static Command testAuto() {
        return Commands
                .sequence(AutoSegments.shootNote(), AutoSegments.scoreNote(new Translation2d(2.87, 4.1)),
                        AutoSegments.scoreNote(new Translation2d(2.87, 5.55)))
                .withName("Test Auto");
    }
}
