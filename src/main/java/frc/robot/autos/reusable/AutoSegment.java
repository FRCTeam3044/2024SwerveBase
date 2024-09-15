package frc.robot.autos.reusable;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.statemachine.reusable.BTrigger;

public class AutoSegment extends Command {
    protected EventLoop loop;
    private boolean isFinished = false;
    private boolean running = false;
    private boolean debug;

    private HashMap<String, Trigger> debugTriggers = new HashMap<>();

    public AutoSegment(String name, boolean debug) {
        this.loop = new EventLoop();
        this.debug = debug;
        this.setName(name);
    }

    public AutoSegment(String name) {
        this(name, false);
    }

    @Override
    public void initialize() {
        if (debug) {
            System.out.println("Starting AutoSegment %s".formatted(this.getName()));
        }
    }

    @Override
    public void execute() {
        running = true;
        loop.poll();
        for (String n : debugTriggers.keySet()) {
            Trigger t = debugTriggers.get(n);
            System.out.println("[%s] Trigger %s is %s".formatted(this.getName(), n, t.getAsBoolean()));
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        running = false;
        if (debug) {
            System.out.println("Ending AutoSegment %s".formatted(this.getName()));
        }
    }

    public Command end() {
        return Commands.runOnce(() -> {
            isFinished = true;
            running = false;
        });
    }

    public Command endAfter(double seconds) {
        return Commands.waitSeconds(seconds).andThen(end());
    }

    public BTrigger autoEnabled() {
        return new BTrigger(this.loop, DriverStation::isAutonomousEnabled);
    }

    public BTrigger started() {
        return new BTrigger(this.loop, () -> running);
    }

    public void debug(String name, BTrigger t, boolean logEveryCycle) {
        if (logEveryCycle)
            debugTriggers.put(name, t);
        try {
            if (t.m_loopField.get(t) != this.loop) {
                System.out.println("Trigger %s is not in the correct loop!".formatted(name));
            }
        } catch (IllegalArgumentException | IllegalAccessException e) {
            e.printStackTrace();
        }
        t.onTrue(Commands
                .runOnce(() -> System.out.println("[%s] Trigger %s switched to true".formatted(this.getName(), name))));
        t.onFalse(Commands.runOnce(
                () -> System.out.println("[%s] Trigger %s switched to false".formatted(this.getName(), name))));
    }
}
