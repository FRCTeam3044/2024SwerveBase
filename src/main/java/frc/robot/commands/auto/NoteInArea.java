package frc.robot.commands.auto;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.NoteDetection;
import me.nabdev.pathfinding.autos.AutoBoolean;

public class NoteInArea implements AutoBoolean {
    private final NoteDetection m_noteDetection;
    private boolean sawNote = false;
    private final boolean usingTargetRegion;
    private final boolean finishIfNone;
    private Pose2d targetRegion;
    private double targetRadius;
    private boolean initialized = false;

    public NoteInArea(NoteDetection m_noteDetection, boolean finishIfNone) {
        this.m_noteDetection = m_noteDetection;
        usingTargetRegion = false;
        this.finishIfNone = finishIfNone;
    }

    public NoteInArea(NoteDetection m_noteDetection, Pose2d targetRegion, double targetRadius,
            boolean finishIfNone) {
        this.m_noteDetection = m_noteDetection;
        this.targetRegion = targetRegion;
        this.targetRadius = targetRadius;
        usingTargetRegion = true;
        this.finishIfNone = finishIfNone;
    }

    public void initialize() {
        if (usingTargetRegion) {
            m_noteDetection.setRegion(targetRegion, targetRadius);
        }
        if ((!usingTargetRegion && m_noteDetection.hasNote) || (usingTargetRegion && m_noteDetection.hasNoteInRegion)) {
            sawNote = true;
        }
    }

    public void execute() {
        if ((!usingTargetRegion && m_noteDetection.hasNote) || (usingTargetRegion && m_noteDetection.hasNoteInRegion)) {
            sawNote = true;
        }
    }

    private boolean sawNote() {
        if (!initialized) {
            initialize();
            initialized = true;
        }
        execute();
        return (!finishIfNone && sawNote) || (finishIfNone && !sawNote);
    }

    @Override
    public BooleanSupplier getSupplier(BooleanSupplier... children) {
        return this::sawNote;
    }
}
