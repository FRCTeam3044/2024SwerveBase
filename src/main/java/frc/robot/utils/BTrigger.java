package frc.robot.utils;

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BTrigger extends Trigger {
    private Field m_loopField;
    private Field m_conditionField;

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
    public Trigger and(BooleanSupplier trigger) {
        try {
            return new BTrigger((EventLoop) m_loopField.get(this),
                    () -> {
                        try {
                            return ((BooleanSupplier) m_conditionField.get(this)).getAsBoolean()
                                    && trigger.getAsBoolean();
                        } catch (IllegalArgumentException | IllegalAccessException e) {
                            e.printStackTrace();
                            return false;
                        }
                    });
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
    public Trigger or(BooleanSupplier trigger) {
        try {
            return new BTrigger((EventLoop) m_loopField.get(this),
                    () -> {
                        try {
                            return ((BooleanSupplier) m_conditionField.get(this)).getAsBoolean()
                                    || trigger.getAsBoolean();
                        } catch (IllegalArgumentException | IllegalAccessException e) {
                            e.printStackTrace();
                            return false;
                        }
                    });
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
    public Trigger negate() {
        try {
            return new BTrigger((EventLoop) m_loopField.get(this),
                    () -> {
                        try {
                            return !((BooleanSupplier) m_conditionField.get(this)).getAsBoolean();
                        } catch (IllegalArgumentException | IllegalAccessException e) {
                            e.printStackTrace();
                            return false;
                        }
                    });
        } catch (IllegalAccessException e) {
            System.out.println("Wpilib is terrible.");
            e.printStackTrace();
            return null;
        }
    }
}
