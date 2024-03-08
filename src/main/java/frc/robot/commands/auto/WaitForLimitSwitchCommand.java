package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.LimitSwitchSubsystem;

public class WaitForLimitSwitchCommand extends Command {
    private final LimitSwitchSubsystem m_Subsystem;
    private final boolean m_WaitForOpen;
    private final Timer m_Timer = new Timer();
    private final double m_Time;
    private boolean startedTimer;

    public WaitForLimitSwitchCommand(LimitSwitchSubsystem subsystem, boolean waitForOpen, double time) {
        m_Subsystem = subsystem;
        m_WaitForOpen = waitForOpen;
        m_Time = time;
    }

    @Override
    public void initialize() {
        m_Timer.reset();
        m_Timer.stop();
        startedTimer = false;
    }

    @Override
    public void execute() {
        if ((m_WaitForOpen && !m_Subsystem.readLimitSwitch()) || (!m_WaitForOpen && m_Subsystem.readLimitSwitch())) {
            if (!startedTimer) {
                m_Timer.start();
                startedTimer = true;
            }
        } else {
            m_Timer.stop();
            m_Timer.reset();
            startedTimer = false;
        }
    }

    @Override
    public boolean isFinished() {
        return m_Timer.get() > m_Time;
    }
}
