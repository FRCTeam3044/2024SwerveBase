package frc.robot.statemachine.reusable;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SmartXboxController {
    private SmartEventLoop loop;
    public CommandXboxController controller;

    public SmartXboxController(CommandXboxController controller, SmartEventLoop loop) {
        this.controller = controller;
        this.loop = loop;
    }

    public XboxController getHID() {
        return controller.getHID();
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @return an event instance representing the left bumper's digital signal
     *         attached to the {@link
     *         CommandScheduler#getDefaultButtonLoop() default scheduler button
     *         loop}.
     * @see #leftBumper(EventLoop)
     */
    public SmartTrigger leftBumper() {
        return SmartTrigger.from(loop, controller.leftBumper());
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @return an event instance representing the right bumper's digital signal
     *         attached to the {@link
     *         CommandScheduler#getDefaultButtonLoop() default scheduler button
     *         loop}.
     * @see #rightBumper(EventLoop)
     */
    public SmartTrigger rightBumper() {
        return SmartTrigger.from(loop, controller.rightBumper());
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @return an event instance representing the left stick button's digital signal
     *         attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     * @see #leftStick(EventLoop)
     */
    public SmartTrigger leftStick() {
        return SmartTrigger.from(loop, controller.leftStick());
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @return an event instance representing the right stick button's digital
     *         signal attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     * @see #rightStick(EventLoop)
     */
    public SmartTrigger rightStick() {
        return SmartTrigger.from(loop, controller.rightStick());
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @return an event instance representing the A button's digital signal attached
     *         to the {@link
     *         CommandScheduler#getDefaultButtonLoop() default scheduler button
     *         loop}.
     * @see #a(EventLoop)
     */
    public SmartTrigger a() {
        return SmartTrigger.from(loop, controller.a());
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @return an event instance representing the B button's digital signal attached
     *         to the {@link
     *         CommandScheduler#getDefaultButtonLoop() default scheduler button
     *         loop}.
     * @see #b(EventLoop)
     */
    public SmartTrigger b() {
        return SmartTrigger.from(loop, controller.b());
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @return an event instance representing the X button's digital signal attached
     *         to the {@link
     *         CommandScheduler#getDefaultButtonLoop() default scheduler button
     *         loop}.
     * @see #x(EventLoop)
     */
    public SmartTrigger x() {
        return SmartTrigger.from(loop, controller.x());
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @return an event instance representing the Y button's digital signal attached
     *         to the {@link
     *         CommandScheduler#getDefaultButtonLoop() default scheduler button
     *         loop}.
     * @see #y(EventLoop)
     */
    public SmartTrigger y() {
        return SmartTrigger.from(loop, controller.y());
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @return an event instance representing the start button's digital signal
     *         attached to the {@link
     *         CommandScheduler#getDefaultButtonLoop() default scheduler button
     *         loop}.
     * @see #start(EventLoop)
     */
    public SmartTrigger start() {
        return SmartTrigger.from(loop, controller.start());
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @return an event instance representing the back button's digital signal
     *         attached to the {@link
     *         CommandScheduler#getDefaultButtonLoop() default scheduler button
     *         loop}.
     * @see #back(EventLoop)
     */
    public SmartTrigger back() {
        return SmartTrigger.from(loop, controller.back());
    }

    /**
     * Constructs a SmartTrigger instance around the axis value of the left trigger.
     * The
     * returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link SmartTrigger}
     *                  to
     *                  be true. This value
     *                  should be in the range [0, 1] where 0 is the unpressed state
     *                  of the axis.
     * @return a SmartTrigger instance that is true when the left trigger's axis
     *         exceeds
     *         the provided
     *         threshold, attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public SmartTrigger leftTrigger(double threshold) {
        return SmartTrigger.from(loop, controller.leftTrigger(threshold));
    }

    /**
     * Constructs a SmartTrigger instance around the axis value of the left trigger.
     * The
     * returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a SmartTrigger instance that is true when the left trigger's axis
     *         exceeds
     *         0.5, attached to
     *         the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public SmartTrigger leftTrigger() {
        return leftTrigger(0.5);
    }

    /**
     * Constructs a SmartTrigger instance around the axis value of the right
     * trigger. The
     * returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link SmartTrigger}
     *                  to
     *                  be true. This value
     *                  should be in the range [0, 1] where 0 is the unpressed state
     *                  of the axis.
     * @return a SmartTrigger instance that is true when the right trigger's axis
     *         exceeds
     *         the provided
     *         threshold, attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public SmartTrigger rightTrigger(double threshold) {
        return SmartTrigger.from(loop, controller.rightTrigger(threshold));
    }

    /**
     * Constructs a SmartTrigger instance around the axis value of the right
     * trigger. The
     * returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a SmartTrigger instance that is true when the right trigger's axis
     *         exceeds
     *         0.5, attached to
     *         the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public SmartTrigger rightTrigger() {
        return rightTrigger(0.5);
    }

    /**
     * Constructs an event instance around this button's digital signal.
     *
     * @param button the button index
     * @return an event instance representing the button's digital signal attached
     *         to the {@link
     *         CommandScheduler#getDefaultButtonLoop() default scheduler button
     *         loop}.
     * @see #button(int, EventLoop)
     */
    public SmartTrigger button(int button) {
        return SmartTrigger.from(loop, controller.button(button));
    }

    /**
     * Constructs a SmartTrigger instance based around this angle of the default
     * (index
     * 0) POV on the HID,
     * attached to {@link CommandScheduler#getDefaultButtonLoop() the default
     * command scheduler button
     * loop}.
     *
     * <p>
     * The POV angles start at 0 in the up direction, and increase clockwise (e.g.
     * right is 90,
     * upper-left is 315).
     *
     * @param angle POV angle in degrees, or -1 for the center / not pressed.
     * @return a SmartTrigger instance based around this angle of a POV on the HID.
     */
    public SmartTrigger pov(int angle) {
        return SmartTrigger.from(loop, controller.pov(angle));
    }

    /**
     * Constructs a SmartTrigger instance based around the 0 degree angle (up) of
     * the
     * default (index 0) POV
     * on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the
     * default command
     * scheduler button loop}.
     *
     * @return a SmartTrigger instance based around the 0 degree angle of a POV on
     *         the
     *         HID.
     */
    public SmartTrigger povUp() {
        return pov(0);
    }

    /**
     * Constructs a SmartTrigger instance based around the 45 degree angle (right
     * up) of
     * the default (index
     * 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop()
     * the default
     * command scheduler button loop}.
     *
     * @return a SmartTrigger instance based around the 45 degree angle of a POV on
     *         the
     *         HID.
     */
    public SmartTrigger povUpRight() {
        return pov(45);
    }

    /**
     * Constructs a SmartTrigger instance based around the 90 degree angle (right)
     * of the
     * default (index 0)
     * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop()
     * the default command
     * scheduler button loop}.
     *
     * @return a SmartTrigger instance based around the 90 degree angle of a POV on
     *         the
     *         HID.
     */
    public SmartTrigger povRight() {
        return pov(90);
    }

    /**
     * Constructs a SmartTrigger instance based around the 135 degree angle (right
     * down)
     * of the default
     * (index 0) POV on the HID, attached to
     * {@link CommandScheduler#getDefaultButtonLoop() the
     * default command scheduler button loop}.
     *
     * @return a SmartTrigger instance based around the 135 degree angle of a POV on
     *         the
     *         HID.
     */
    public SmartTrigger povDownRight() {
        return pov(135);
    }

    /**
     * Constructs a SmartTrigger instance based around the 180 degree angle (down)
     * of the
     * default (index 0)
     * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop()
     * the default command
     * scheduler button loop}.
     *
     * @return a SmartTrigger instance based around the 180 degree angle of a POV on
     *         the
     *         HID.
     */
    public SmartTrigger povDown() {
        return pov(180);
    }

    /**
     * Constructs a SmartTrigger instance based around the 225 degree angle (down
     * left)
     * of the default
     * (index 0) POV on the HID, attached to
     * {@link CommandScheduler#getDefaultButtonLoop() the
     * default command scheduler button loop}.
     *
     * @return a SmartTrigger instance based around the 225 degree angle of a POV on
     *         the
     *         HID.
     */
    public SmartTrigger povDownLeft() {
        return pov(225);
    }

    /**
     * Constructs a SmartTrigger instance based around the 270 degree angle (left)
     * of the
     * default (index 0)
     * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop()
     * the default command
     * scheduler button loop}.
     *
     * @return a SmartTrigger instance based around the 270 degree angle of a POV on
     *         the
     *         HID.
     */
    public SmartTrigger povLeft() {
        return pov(270);
    }

    /**
     * Constructs a SmartTrigger instance based around the 315 degree angle (left
     * up) of
     * the default (index
     * 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop()
     * the default
     * command scheduler button loop}.
     *
     * @return a SmartTrigger instance based around the 315 degree angle of a POV on
     *         the
     *         HID.
     */
    public SmartTrigger povUpLeft() {
        return pov(315);
    }

    /**
     * Constructs a SmartTrigger instance based around the center (not pressed)
     * position
     * of the default
     * (index 0) POV on the HID, attached to
     * {@link CommandScheduler#getDefaultButtonLoop() the
     * default command scheduler button loop}.
     *
     * @return a SmartTrigger instance based around the center position of a POV on
     *         the
     *         HID.
     */
    public SmartTrigger povCenter() {
        return pov(-1);
    }

    /**
     * Constructs a SmartTrigger instance that is true when the axis value is less
     * than
     * {@code threshold},
     * attached to {@link CommandScheduler#getDefaultButtonLoop() the default
     * command scheduler button
     * loop}.
     *
     * @param axis      The axis to read, starting at 0
     * @param threshold The value below which this trigger should return true.
     * @return a SmartTrigger instance that is true when the axis value is less than
     *         the
     *         provided
     *         threshold.
     */
    public SmartTrigger axisLessThan(int axis, double threshold) {
        return SmartTrigger.from(loop, controller.axisLessThan(axis, threshold));
    }

    /**
     * Constructs a SmartTrigger instance that is true when the axis value is less
     * than
     * {@code threshold},
     * attached to {@link CommandScheduler#getDefaultButtonLoop() the default
     * command scheduler button
     * loop}.
     *
     * @param axis      The axis to read, starting at 0
     * @param threshold The value above which this trigger should return true.
     * @return a SmartTrigger instance that is true when the axis value is greater
     *         than
     *         the provided
     *         threshold.
     */
    public SmartTrigger axisGreaterThan(int axis, double threshold) {
        return SmartTrigger.from(loop, controller.axisGreaterThan(axis, threshold));
    }
}
