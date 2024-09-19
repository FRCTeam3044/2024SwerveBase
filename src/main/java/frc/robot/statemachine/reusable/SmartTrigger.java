// Adapted from wpilib Trigger class
// Copyright (c) 2009-2024 FIRST and other WPILib contributors All rights reserved.

package frc.robot.statemachine.reusable;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

public class SmartTrigger implements BooleanSupplier {
    private final BooleanSupplier m_condition;
    private final SmartEventLoop m_loop;

    /**
     * Creates a new trigger based on the given condition.
     *
     * @param loop      The loop instance that polls this trigger
     * @param condition the condition represented by this trigger
     */
    public SmartTrigger(SmartEventLoop loop, BooleanSupplier condition) {
        m_loop = requireNonNullParam(loop, "loop", "StateTrigger");
        m_condition = requireNonNullParam(condition, "condition", "StateTrigger");
    }

    /**
     * Constructs a new SmartTrigger from a loop and condition.
     * 
     * @param loop      The loop instance that polls this trigger
     * @param condition the condition represented by this trigger
     */
    public static SmartTrigger from(SmartEventLoop loop, BooleanSupplier condition) {
        return new SmartTrigger(loop, condition);
    }

    /**
     * Starts the given command whenever the condition changes from `false` to
     * `true`.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public SmartTrigger onTrue(Command command) {
        requireNonNullParam(command, "command", "onTrue");
        m_loop.bind(command,
                new Runnable() {
                    private boolean m_pressedLast = m_condition.getAsBoolean();

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (!m_pressedLast && pressed) {
                            command.schedule();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Starts the given command whenever the condition changes from `true` to
     * `false`.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public SmartTrigger onFalse(Command command) {
        requireNonNullParam(command, "command", "onFalse");
        m_loop.bind(command,
                new Runnable() {
                    private boolean m_pressedLast = m_condition.getAsBoolean();

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (m_pressedLast && !pressed) {
                            command.schedule();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Starts the given command when the condition is `true` and keeps it running
     * until the condition is `false`.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public SmartTrigger runWhileTrue(Command command) {
        requireNonNullParam(command, "command", "whileTrue");
        m_loop.bind(command,
                new Runnable() {
                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (!command.isScheduled() && pressed) {
                            command.schedule();
                        } else if (command.isScheduled() && !pressed) {
                            command.cancel();
                        }
                    }
                });
        return this;
    }

    /**
     * Starts the given command when the condition is `false` and keeps it running
     * until the condition is `true`.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public SmartTrigger runWhileFalse(Command command) {
        requireNonNullParam(command, "command", "whileFalse");
        m_loop.bind(command,
                new Runnable() {
                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (!command.isScheduled() && !pressed) {
                            command.schedule();
                        } else if (command.isScheduled() && pressed) {
                            command.cancel();
                        }
                    }
                });
        return this;
    }

    /**
     * Starts the given command when the condition changes to `true` and cancels it
     * when the condition changes to `false`.
     *
     * <p>
     * Doesn't re-start the command if it ends while the condition is still `true`.
     * If the command
     * should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public SmartTrigger whileTrue(Command command) {
        requireNonNullParam(command, "command", "whileTrue");
        m_loop.bind(command,
                new Runnable() {
                    private boolean m_pressedLast = m_condition.getAsBoolean();

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (!m_pressedLast && pressed) {
                            command.schedule();
                        } else if (m_pressedLast && !pressed) {
                            command.cancel();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Starts the given command when the condition changes to `false` and cancels it
     * when the condition changes to `true`.
     *
     * <p>
     * Doesn't re-start the command if it ends while the condition is still `false`.
     * If the command
     * should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public SmartTrigger whileFalse(Command command) {
        requireNonNullParam(command, "command", "whileFalse");
        m_loop.bind(command,
                new Runnable() {
                    private boolean m_pressedLast = m_condition.getAsBoolean();

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (m_pressedLast && !pressed) {
                            command.schedule();
                        } else if (!m_pressedLast && pressed) {
                            command.cancel();
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Toggles a command when the condition changes from `false` to `true`.
     *
     * @param command the command to toggle
     * @return this trigger, so calls can be chained
     */
    public SmartTrigger toggleOnTrue(Command command) {
        requireNonNullParam(command, "command", "toggleOnTrue");
        m_loop.bind(command,
                new Runnable() {
                    private boolean m_pressedLast = m_condition.getAsBoolean();

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (!m_pressedLast && pressed) {
                            if (command.isScheduled()) {
                                command.cancel();
                            } else {
                                command.schedule();
                            }
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    /**
     * Toggles a command when the condition changes from `true` to `false`.
     *
     * @param command the command to toggle
     * @return this trigger, so calls can be chained
     */
    public SmartTrigger toggleOnFalse(Command command) {
        requireNonNullParam(command, "command", "toggleOnFalse");
        m_loop.bind(command,
                new Runnable() {
                    private boolean m_pressedLast = m_condition.getAsBoolean();

                    @Override
                    public void run() {
                        boolean pressed = m_condition.getAsBoolean();

                        if (m_pressedLast && !pressed) {
                            if (command.isScheduled()) {
                                command.cancel();
                            } else {
                                command.schedule();
                            }
                        }

                        m_pressedLast = pressed;
                    }
                });
        return this;
    }

    @Override
    public boolean getAsBoolean() {
        return m_condition.getAsBoolean();
    }

    /**
     * Composes two triggers with logical AND.
     *
     * @param trigger the condition to compose with
     * @return A trigger which is active when both component triggers are active.
     */
    public SmartTrigger and(BooleanSupplier trigger) {
        return new SmartTrigger(m_loop, () -> m_condition.getAsBoolean() && trigger.getAsBoolean());
    }

    /**
     * Composes two triggers with logical OR.
     *
     * @param trigger the condition to compose with
     * @return A trigger which is active when either component trigger is active.
     */
    public SmartTrigger or(BooleanSupplier trigger) {
        return new SmartTrigger(m_loop, () -> m_condition.getAsBoolean() || trigger.getAsBoolean());
    }

    /**
     * Creates a new trigger that is active when this trigger is inactive, i.e. that
     * acts as the
     * negation of this trigger.
     *
     * @return the negated trigger
     */
    public SmartTrigger negate() {
        return new SmartTrigger(m_loop, () -> !m_condition.getAsBoolean());
    }

    /**
     * Creates a new debounced trigger from this trigger - it will become active
     * when this trigger has
     * been active for longer than the specified period.
     *
     * @param seconds The debounce period.
     * @return The debounced trigger (rising edges debounced only)
     */
    public SmartTrigger debounce(double seconds) {
        return debounce(seconds, Debouncer.DebounceType.kRising);
    }

    /**
     * Creates a new debounced trigger from this trigger - it will become active
     * when this trigger has
     * been active for longer than the specified period.
     *
     * @param seconds The debounce period.
     * @param type    The debounce type.
     * @return The debounced trigger.
     */
    public SmartTrigger debounce(double seconds, Debouncer.DebounceType type) {
        return new SmartTrigger(m_loop,
                new BooleanSupplier() {
                    final Debouncer m_debouncer = new Debouncer(seconds, type);

                    @Override
                    public boolean getAsBoolean() {
                        return m_debouncer.calculate(m_condition.getAsBoolean());
                    }
                });
    }
}
