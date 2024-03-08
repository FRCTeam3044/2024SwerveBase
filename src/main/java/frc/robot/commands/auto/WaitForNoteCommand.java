package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteDetection;

public class WaitForNoteCommand extends Command {
    private final NoteDetection m_noteDetection;
    private boolean sawNote = false;
    private final boolean usingTargetRegion;
    private final boolean finishIfNone;
    private Pose2d targetRegion;
    private double targetRadius;

    public WaitForNoteCommand(NoteDetection m_noteDetection, boolean finishIfNone) {
        this.m_noteDetection = m_noteDetection;
        usingTargetRegion = false;
        this.finishIfNone = finishIfNone;
    }

    public WaitForNoteCommand(NoteDetection m_noteDetection, Pose2d targetRegion, double targetRadius,
            boolean finishIfNone) {
        this.m_noteDetection = m_noteDetection;
        this.targetRegion = targetRegion;
        this.targetRadius = targetRadius;
        usingTargetRegion = true;
        this.finishIfNone = finishIfNone;
    }

    @Override
    public void initialize() {
        if (usingTargetRegion) {
            m_noteDetection.setRegion(targetRegion, targetRadius);
        }
        if ((!usingTargetRegion && m_noteDetection.hasNote) || (usingTargetRegion && m_noteDetection.hasNoteInRegion)) {
            sawNote = true;
        }
    }

    @Override
    public void execute() {
        if ((!usingTargetRegion && m_noteDetection.hasNote) || (usingTargetRegion && m_noteDetection.hasNoteInRegion)) {
            sawNote = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (!finishIfNone && sawNote) || (finishIfNone && !sawNote);
    }
}
