package frc.robot.utils;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * BTrigger = Better Trigger
 * 
 * WPILib triggers with some extra functionality.
 */
public class BTrigger extends Trigger {
    public ArrayList<Command> managedCommands = new ArrayList<>();

    public Field m_loopField;
    private Field m_conditionField;

    /**
     * Stop all command associated with this trigger.
     */
    public void stopCommands() {
        for (Command c : managedCommands) {
            c.cancel();
        }
    }

    @Override
    public BTrigger onTrue(Command c) {
        super.onTrue(c);
        managedCommands.add(c);
        return this;
    }

    @Override
    public BTrigger onFalse(Command c) {
        super.onFalse(c);
        managedCommands.add(c);
        return this;
    }

    @Override
    public BTrigger whileTrue(Command c) {
        super.whileTrue(c);
        managedCommands.add(c);
        return this;
    }

    @Override
    public BTrigger whileFalse(Command c) {
        super.whileFalse(c);
        managedCommands.add(c);
        return this;
    }

    @Override
    public BTrigger toggleOnTrue(Command c) {
        super.toggleOnTrue(c);
        managedCommands.add(c);
        return this;
    }

    @Override
    public BTrigger toggleOnFalse(Command c) {
        super.toggleOnFalse(c);
        managedCommands.add(c);
        return this;
    }

    /**
     * Creates a new trigger based on the given condition.
     *
     * @param loop      The loop instance that polls this trigger.
     * @param condition the condition represented by this trigger
     */
    public BTrigger(EventLoop loop, BooleanSupplier condition) {
        super(loop, condition);
        try {
            m_loopField = Trigger.class.getDeclaredField("m_loop");
            m_loopField.setAccessible(true);
            m_conditionField = Trigger.class.getDeclaredField("m_condition");
            m_conditionField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        }
    }

    /**
     * Creates a new trigger based on the given condition.
     *
     * <p>
     * Polled by the default scheduler button loop.
     *
     * @param condition the condition represented by this trigger
     */
    public BTrigger(BooleanSupplier condition) {
        super(condition);
        try {
            m_loopField = Trigger.class.getDeclaredField("m_loop");
            m_loopField.setAccessible(true);
            m_conditionField = Trigger.class.getDeclaredField("m_condition");
            m_conditionField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        }
    }

    /**
     * Composes two triggers with logical AND.
     *
     * @param trigger the condition to compose with
     * @return A trigger which is active when both component triggers are active.
     */
    public BTrigger and(BooleanSupplier trigger) {
        try {
            BooleanSupplier thisSupplier = (BooleanSupplier) m_conditionField.get(this);
            return new BTrigger((EventLoop) m_loopField.get(this),
                    () -> thisSupplier.getAsBoolean() && trigger.getAsBoolean());
        } catch (IllegalAccessException e) {
            System.out.println("Wpilib is terrible.");
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Composes two triggers with logical OR.
     *
     * @param trigger the condition to compose with
     * @return A trigger which is active when either component trigger is active.
     */
    public BTrigger or(BooleanSupplier trigger) {
        try {
            BooleanSupplier thisSupplier = (BooleanSupplier) m_conditionField.get(this);
            return new BTrigger((EventLoop) m_loopField.get(this),
                    () -> thisSupplier.getAsBoolean() || trigger.getAsBoolean());
        } catch (IllegalAccessException e) {
            System.out.println("Wpilib is terrible.");
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Creates a new trigger that is active when this trigger is inactive, i.e. that
     * acts as the
     * negation of this trigger.
     *
     * @return the negated trigger
     */
    public BTrigger negate() {
        try {
            BooleanSupplier thisSupplier = (BooleanSupplier) m_conditionField.get(this);
            return new BTrigger((EventLoop) m_loopField.get(this), () -> !thisSupplier.getAsBoolean());
        } catch (IllegalAccessException e) {
            System.out.println("Wpilib is terrible.");
            e.printStackTrace();
            return null;
        }
    }
}
