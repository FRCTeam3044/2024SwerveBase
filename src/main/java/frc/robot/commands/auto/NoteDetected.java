package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.NoteDetection;
import me.nabdev.pathfinding.autos.AutoBoolean;

public class NoteDetected implements AutoBoolean {
    private final NoteDetection mDetection;

    public NoteDetected(NoteDetection noteDetection) {
        this.mDetection = noteDetection;
    }

    @Override
    public BooleanSupplier getSupplier(BooleanSupplier... children) {
        return this::noteDetected;
    }

    private boolean noteDetected() {
        return mDetection.hasNote;
    }

}
