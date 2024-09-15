// Adapted from wpilib EventLoop class
// Copyright (c) 2009-2024 FIRST and other WPILib contributors All rights reserved.

package frc.robot.statemachine.reusable;

import java.util.Collection;
import java.util.ConcurrentModificationException;
import java.util.LinkedHashSet;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A declarative way to bind a set of actions to a loop and execute them when
 * the loop is polled.
 */
public final class SmartEventLoop {
    private final Collection<Runnable> m_bindings = new LinkedHashSet<>();
    private final Collection<Command> m_commands = new LinkedHashSet<>();
    private boolean m_running;

    /**
     * Bind a new action to run when the loop is polled.
     *
     * @param action the action to run.
     */
    public void bind(Command command, Runnable action) {
        if (m_running) {
            throw new ConcurrentModificationException("Cannot bind SmartEventLoop while it is running");
        }
        m_commands.add(command);
        m_bindings.add(action);
    }

    /** Poll all bindings. */
    public void poll() {
        m_running = true;
        m_bindings.forEach(Runnable::run);
    }

    /** Clear all bindings. */
    public void clear() {
        if (m_running) {
            throw new ConcurrentModificationException("Cannot clear SmartEventLoop while it is running");
        }
        m_bindings.clear();
    }

    /** Stops the event loop and cancels all associated commands. */
    public void stop() {
        m_running = false;
        m_commands.forEach(Command::cancel);
    }

}
