package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteDetection;

public class WaitForNoteCommand extends Command {
    private final NoteDetection m_noteDetection;
    private boolean sawNote = false;
    private final boolean usingTargetRegion;
    private Pose2d targetRegion;
    private double targetRadius;

    public WaitForNoteCommand(NoteDetection m_noteDetection) {
        this.m_noteDetection = m_noteDetection;
        usingTargetRegion = false;
    }

    public WaitForNoteCommand(NoteDetection m_noteDetection, Pose2d targetRegion, double targetRadius) {
        this.m_noteDetection = m_noteDetection;
        this.targetRegion = targetRegion;
        this.targetRadius = targetRadius;
        usingTargetRegion = true;
    }

    @Override
    public void initialize() {
        if (usingTargetRegion) {
            m_noteDetection.setRegion(targetRegion, targetRadius);
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
        return sawNote;
    }
}
