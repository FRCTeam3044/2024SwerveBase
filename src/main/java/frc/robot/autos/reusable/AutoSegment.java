package frc.robot.autos.reusable;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutoSegment extends Command {
    protected EventLoop loop;
    private boolean isFinished = false;

    public AutoSegment() {
        this.loop = new EventLoop();
    }

    @Override
    public void execute() {
        loop.poll();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    public Command end() {
        return Commands.runOnce(() -> isFinished = true);
    }

    public Command endAfter(double seconds) {
        return Commands.waitSeconds(seconds).andThen(end());
    }

    public Trigger autoEnabled() {
        return new Trigger(this.loop, DriverStation::isAutonomousEnabled);
    }
}
