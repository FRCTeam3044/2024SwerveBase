package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteDetection;

public class WaitForNoteCommand extends Command {
    private final NoteDetection m_noteDetection;
    private boolean sawNote = false;

    public WaitForNoteCommand(NoteDetection m_noteDetection) {
        this.m_noteDetection = m_noteDetection;
    }

    @Override
    public void execute() {
        if (m_noteDetection.hasNote) {
            sawNote = true;
        }
    }

    @Override
    public boolean isFinished() {
        return sawNote;
    }
}
