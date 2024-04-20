// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.lib2706.button;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.lib.lib2706.button.FakeCommandXboxController.FakeXboxController.Axis;
import frc.lib.lib2706.button.FakeCommandXboxController.FakeXboxController.Button;

/**
 * A version of {@link XboxController} with {@link Trigger} factories for
 * command-based.
 *
 * @see XboxController
 */
@SuppressWarnings("MethodName")
public class FakeCommandXboxController extends CommandXboxController {
    public enum FakeControllerType {
        Custom,
        LogitechDualAction,
        EliminatorAfterShock,
    }

    private final FakeXboxController m_hid;
    private final FakeControllerType m_type;
    private final double BUTTON_TRIGGER_VALUE =
            0.6; // The value to fake when a trigger button is pressed

    public FakeCommandXboxController(int port, FakeControllerType type) {
        super(port);
        m_type = type;

        Button buttons = new Button();
        Axis axes = new Axis();
        switch (type) {
            case LogitechDualAction:
                buttons.kX = 1;
                buttons.kA = 2;
                buttons.kB = 3;
                buttons.kY = 4;
                buttons.kLeftBumper = 5;
                buttons.kRightBumper = 6;
                buttons.kBack = 9;
                buttons.kStart = 10;
                buttons.kLeftStick = 11;
                buttons.kRightStick = 12;

                axes.kLeftX = 0;
                axes.kLeftY = 1;
                axes.kRightX = 2;
                axes.kRightY = 3;
                axes.kLeftTrigger = 7; // This is a button not an axis
                axes.kRightTrigger = 8; // This is a button not an axis
                axes.kTriggersAreButtons = true;
                break;
            case EliminatorAfterShock:
                buttons.kX = 1;
                buttons.kA = 2;
                buttons.kB = 3;
                buttons.kY = 4;
                buttons.kLeftBumper = 5;
                buttons.kRightBumper = 6;
                buttons.kBack = 9;
                buttons.kStart = 10;
                buttons.kLeftStick = -1;
                buttons.kRightStick = -1;

                axes.kLeftX = 0;
                axes.kLeftY = 1;
                axes.kRightX = 4;
                axes.kRightY = 2;
                axes.kLeftTrigger = 7; // This is a button not an axis
                axes.kRightTrigger = 8; // This is a button not an axis
                axes.kTriggersAreButtons = true;
                break;
            case Custom:
            default:
                break;
        }

        m_hid = new FakeXboxController(port, buttons, axes);
    }

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is
     *             plugged into.
     */
    public FakeCommandXboxController(int port, Button buttons, Axis axes) {
        super(port);
        m_type = FakeControllerType.Custom;
        m_hid = new FakeXboxController(port, buttons, axes);
    }

    /**
     * Get the underlying GenericHID object.
     *
     * @return the wrapped GenericHID object
     */
    @Override
    public XboxController getHID() {
        return m_hid;
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
    public Trigger leftBumper() {
        return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right bumper's digital signal
     *         attached to the given
     *         loop.
     */
    public Trigger leftBumper(EventLoop loop) {
        return m_hid.leftBumper(loop).castTo(Trigger::new);
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
    public Trigger rightBumper() {
        return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left bumper's digital signal
     *         attached to the given
     *         loop.
     */
    public Trigger rightBumper(EventLoop loop) {
        return m_hid.rightBumper(loop).castTo(Trigger::new);
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
    public Trigger leftStick() {
        return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left stick button's digital signal
     *         attached to the
     *         given loop.
     */
    public Trigger leftStick(EventLoop loop) {
        return m_hid.leftStick(loop).castTo(Trigger::new);
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
    public Trigger rightStick() {
        return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right stick button's digital
     *         signal attached to the
     *         given loop.
     */
    public Trigger rightStick(EventLoop loop) {
        return m_hid.rightStick(loop).castTo(Trigger::new);
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
    public Trigger a() {
        return a(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the A button's digital signal attached
     *         to the given
     *         loop.
     */
    public Trigger a(EventLoop loop) {
        return m_hid.a(loop).castTo(Trigger::new);
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
    public Trigger b() {
        return b(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the B button's digital signal attached
     *         to the given
     *         loop.
     */
    public Trigger b(EventLoop loop) {
        return m_hid.b(loop).castTo(Trigger::new);
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
    public Trigger x() {
        return x(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the X button's digital signal attached
     *         to the given
     *         loop.
     */
    public Trigger x(EventLoop loop) {
        return m_hid.x(loop).castTo(Trigger::new);
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
    public Trigger y() {
        return y(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Y button's digital signal attached
     *         to the given
     *         loop.
     */
    public Trigger y(EventLoop loop) {
        return m_hid.y(loop).castTo(Trigger::new);
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
    public Trigger start() {
        return start(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's digital signal
     *         attached to the given
     *         loop.
     */
    public Trigger start(EventLoop loop) {
        return m_hid.start(loop).castTo(Trigger::new);
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
    public Trigger back() {
        return back(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the back button's digital signal
     *         attached to the given
     *         loop.
     */
    public Trigger back(EventLoop loop) {
        return m_hid.back(loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The
     * returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to
     *                  be true. This value
     *                  should be in the range [0, 1] where 0 is the unpressed state
     *                  of the axis.
     * @param loop      the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the left trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public Trigger leftTrigger(double threshold, EventLoop loop) {
        return m_hid.leftTrigger(threshold, loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The
     * returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to
     *                  be true. This value
     *                  should be in the range [0, 1] where 0 is the unpressed state
     *                  of the axis.
     * @return a Trigger instance that is true when the left trigger's axis exceeds
     *         the provided
     *         threshold, attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger leftTrigger(double threshold) {
        return leftTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The
     * returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the left trigger's axis exceeds
     *         0.5, attached to
     *         the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger leftTrigger() {
        return leftTrigger(0.5);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The
     * returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to
     *                  be true. This value
     *                  should be in the range [0, 1] where 0 is the unpressed state
     *                  of the axis.
     * @param loop      the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the right trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public Trigger rightTrigger(double threshold, EventLoop loop) {
        return m_hid.rightTrigger(threshold, loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The
     * returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to
     *                  be true. This value
     *                  should be in the range [0, 1] where 0 is the unpressed state
     *                  of the axis.
     * @return a Trigger instance that is true when the right trigger's axis exceeds
     *         the provided
     *         threshold, attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger rightTrigger(double threshold) {
        return rightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The
     * returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the right trigger's axis exceeds
     *         0.5, attached to
     *         the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger rightTrigger() {
        return rightTrigger(0.5);
    }

    /**
     * Get the X axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftX() {
        return m_hid.getLeftX();
    }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightX() {
        return m_hid.getRightX();
    }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        return m_hid.getLeftY();
    }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightY() {
        return m_hid.getRightY();
    }

    /**
     * Get the left trigger (LT) axis value of the controller. Note that this axis
     * is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getLeftTriggerAxis() {
        return m_hid.getLeftTriggerAxis();
    }

    /**
     * Get the right trigger (RT) axis value of the controller. Note that this axis
     * is bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getRightTriggerAxis() {
        return m_hid.getRightTriggerAxis();
    }

    /**
     *
     * Nested class to fake the XboxController class is such a way the values can be
     * changed
     *
     */
    public class FakeXboxController extends XboxController {
        public Button mappedButton = new Button();
        public Axis mappedAxis = new Axis();

        /** Represents a digital button on an FakeXboxController. */
        public static class Button {
            public int kLeftBumper = 5 + 1;
            public int kRightBumper = 6 + 1;
            public int kLeftStick = 9 + 1;
            public int kRightStick = 10 + 1;
            public int kA = 1 + 1;
            public int kB = 2 + 1;
            public int kX = 3 + 1;
            public int kY = 4 + 1;
            public int kBack = 7 + 1;
            public int kStart = 8 + 1;
        }

        /** Represents an axis on an FakeXboxController. */
        public static class Axis {
            public int kLeftX = 0 + 1;
            public int kRightX = 4 + 1;
            public int kLeftY = 1 + 1;
            public int kRightY = 5 + 1;
            public int kLeftTrigger = 2 + 1;
            public int kRightTrigger = 3 + 1;
            public boolean kTriggersAreButtons = false;
        }

        /**
         * Construct an instance of a controller.
         *
         * @param port The port index on the Driver Station that the controller is
         *             plugged into.
         */
        public FakeXboxController(final int port, Button mappedButton, Axis mappedAxes) {
            super(port);

            this.mappedButton = mappedButton;
            this.mappedAxis = mappedAxes;

            HAL.report(tResourceType.kResourceType_XboxController, port + 1);
        }

        /**
         * Get the X axis value of left side of the controller.
         *
         * @return The axis value.
         */
        public double getLeftX() {
            return getRawAxis(mappedAxis.kLeftX);
        }

        /**
         * Get the X axis value of right side of the controller.
         *
         * @return The axis value.
         */
        public double getRightX() {
            return getRawAxis(mappedAxis.kRightX);
        }

        /**
         * Get the Y axis value of left side of the controller.
         *
         * @return The axis value.
         */
        public double getLeftY() {
            return getRawAxis(mappedAxis.kLeftY);
        }

        /**
         * Get the Y axis value of right side of the controller.
         *
         * @return The axis value.
         */
        public double getRightY() {
            return getRawAxis(mappedAxis.kRightY);
        }

        /**
         * Get the left trigger (LT) axis value of the controller. Note that this axis
         * is bound to the
         * range of [0, 1] as opposed to the usual [-1, 1].
         *
         * @return The axis value.
         */
        public double getLeftTriggerAxis() {
            if (m_type.equals(FakeControllerType.LogitechDualAction)) {
                return getRawButton(mappedAxis.kLeftTrigger) ? BUTTON_TRIGGER_VALUE : 0;
            } else {
                return getRawAxis(mappedAxis.kLeftTrigger);
            }
        }

        /**
         * Get the right trigger (RT) axis value of the controller. Note that this axis
         * is bound to the
         * range of [0, 1] as opposed to the usual [-1, 1].
         *
         * @return The axis value.
         */
        public double getRightTriggerAxis() {
            if (m_type.equals(FakeControllerType.LogitechDualAction)) {
                return getRawButton(mappedAxis.kRightTrigger) ? BUTTON_TRIGGER_VALUE : 0;
            } else {
                return getRawAxis(mappedAxis.kRightTrigger);
            }
        }

        /**
         * Read the value of the left bumper (LB) button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getLeftBumper() {
            return getRawButton(mappedButton.kLeftBumper);
        }

        /**
         * Read the value of the right bumper (RB) button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getRightBumper() {
            return getRawButton(mappedButton.kRightBumper);
        }

        /**
         * Whether the left bumper (LB) was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getLeftBumperPressed() {
            return getRawButtonPressed(mappedButton.kLeftBumper);
        }

        /**
         * Whether the right bumper (RB) was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getRightBumperPressed() {
            return getRawButtonPressed(mappedButton.kRightBumper);
        }

        /**
         * Whether the left bumper (LB) was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getLeftBumperReleased() {
            return getRawButtonReleased(mappedButton.kLeftBumper);
        }

        /**
         * Whether the right bumper (RB) was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getRightBumperReleased() {
            return getRawButtonReleased(mappedButton.kRightBumper);
        }

        /**
         * Constructs an event instance around the right bumper's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance representing the right bumper's digital signal
         *         attached to the given
         *         loop.
         */
        public BooleanEvent leftBumper(EventLoop loop) {
            return new BooleanEvent(loop, this::getLeftBumper);
        }

        /**
         * Constructs an event instance around the left bumper's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance representing the left bumper's digital signal
         *         attached to the given
         *         loop.
         */
        public BooleanEvent rightBumper(EventLoop loop) {
            return new BooleanEvent(loop, this::getRightBumper);
        }

        /**
         * Read the value of the left stick button (LSB) on the controller.
         *
         * @return The state of the button.
         */
        public boolean getLeftStickButton() {
            return getRawButton(mappedButton.kLeftStick);
        }

        /**
         * Read the value of the right stick button (RSB) on the controller.
         *
         * @return The state of the button.
         */
        public boolean getRightStickButton() {
            return getRawButton(mappedButton.kRightStick);
        }

        /**
         * Whether the left stick button (LSB) was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getLeftStickButtonPressed() {
            return getRawButtonPressed(mappedButton.kLeftStick);
        }

        /**
         * Whether the right stick button (RSB) was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getRightStickButtonPressed() {
            return getRawButtonPressed(mappedButton.kRightStick);
        }

        /**
         * Whether the left stick button (LSB) was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getLeftStickButtonReleased() {
            return getRawButtonReleased(mappedButton.kLeftStick);
        }

        /**
         * Whether the right stick (RSB) button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getRightStickButtonReleased() {
            return getRawButtonReleased(mappedButton.kRightStick);
        }

        /**
         * Constructs an event instance around the left stick button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance representing the left stick button's digital signal
         *         attached to the
         *         given loop.
         */
        public BooleanEvent leftStick(EventLoop loop) {
            return new BooleanEvent(loop, this::getLeftStickButton);
        }

        /**
         * Constructs an event instance around the right stick button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance representing the right stick button's digital
         *         signal attached to the
         *         given loop.
         */
        public BooleanEvent rightStick(EventLoop loop) {
            return new BooleanEvent(loop, this::getRightStickButton);
        }

        /**
         * Read the value of the A button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getAButton() {
            return getRawButton(mappedButton.kA);
        }

        /**
         * Whether the A button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getAButtonPressed() {
            return getRawButtonPressed(mappedButton.kA);
        }

        /**
         * Whether the A button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getAButtonReleased() {
            return getRawButtonReleased(mappedButton.kA);
        }

        /**
         * Constructs an event instance around the A button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance representing the A button's digital signal attached
         *         to the given
         *         loop.
         */
        @SuppressWarnings("MethodName")
        public BooleanEvent a(EventLoop loop) {
            return new BooleanEvent(loop, this::getAButton);
        }

        /**
         * Read the value of the B button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getBButton() {
            return getRawButton(mappedButton.kB);
        }

        /**
         * Whether the B button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getBButtonPressed() {
            return getRawButtonPressed(mappedButton.kB);
        }

        /**
         * Whether the B button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getBButtonReleased() {
            return getRawButtonReleased(mappedButton.kB);
        }

        /**
         * Constructs an event instance around the B button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance representing the B button's digital signal attached
         *         to the given
         *         loop.
         */
        @SuppressWarnings("MethodName")
        public BooleanEvent b(EventLoop loop) {
            return new BooleanEvent(loop, this::getBButton);
        }

        /**
         * Read the value of the X button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getXButton() {
            return getRawButton(mappedButton.kX);
        }

        /**
         * Whether the X button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getXButtonPressed() {
            return getRawButtonPressed(mappedButton.kX);
        }

        /**
         * Whether the X button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getXButtonReleased() {
            return getRawButtonReleased(mappedButton.kX);
        }

        /**
         * Constructs an event instance around the X button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance representing the X button's digital signal attached
         *         to the given
         *         loop.
         */
        @SuppressWarnings("MethodName")
        public BooleanEvent x(EventLoop loop) {
            return new BooleanEvent(loop, this::getXButton);
        }

        /**
         * Read the value of the Y button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getYButton() {
            return getRawButton(mappedButton.kY);
        }

        /**
         * Whether the Y button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getYButtonPressed() {
            return getRawButtonPressed(mappedButton.kY);
        }

        /**
         * Whether the Y button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getYButtonReleased() {
            return getRawButtonReleased(mappedButton.kY);
        }

        /**
         * Constructs an event instance around the Y button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance representing the Y button's digital signal attached
         *         to the given
         *         loop.
         */
        @SuppressWarnings("MethodName")
        public BooleanEvent y(EventLoop loop) {
            return new BooleanEvent(loop, this::getYButton);
        }

        /**
         * Read the value of the back button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getBackButton() {
            return getRawButton(mappedButton.kBack);
        }

        /**
         * Whether the back button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getBackButtonPressed() {
            return getRawButtonPressed(mappedButton.kBack);
        }

        /**
         * Whether the back button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getBackButtonReleased() {
            return getRawButtonReleased(mappedButton.kBack);
        }

        /**
         * Constructs an event instance around the back button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance representing the back button's digital signal
         *         attached to the given
         *         loop.
         */
        public BooleanEvent back(EventLoop loop) {
            return new BooleanEvent(loop, this::getBackButton);
        }

        /**
         * Read the value of the start button on the controller.
         *
         * @return The state of the button.
         */
        public boolean getStartButton() {
            return getRawButton(mappedButton.kStart);
        }

        /**
         * Whether the start button was pressed since the last check.
         *
         * @return Whether the button was pressed since the last check.
         */
        public boolean getStartButtonPressed() {
            return getRawButtonPressed(mappedButton.kStart);
        }

        /**
         * Whether the start button was released since the last check.
         *
         * @return Whether the button was released since the last check.
         */
        public boolean getStartButtonReleased() {
            return getRawButtonReleased(mappedButton.kStart);
        }

        /**
         * Constructs an event instance around the start button's digital signal.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance representing the start button's digital signal
         *         attached to the given
         *         loop.
         */
        public BooleanEvent start(EventLoop loop) {
            return new BooleanEvent(loop, this::getStartButton);
        }

        /**
         * Constructs an event instance around the axis value of the left trigger. The
         * returned trigger
         * will be true when the axis value is greater than {@code threshold}.
         *
         * @param threshold the minimum axis value for the returned {@link BooleanEvent}
         *                  to be true. This
         *                  value should be in the range [0, 1] where 0 is the unpressed
         *                  state of the axis.
         * @param loop      the event loop instance to attach the event to.
         * @return an event instance that is true when the left trigger's axis exceeds
         *         the provided
         *         threshold, attached to the given event loop
         */
        public BooleanEvent leftTrigger(double threshold, EventLoop loop) {
            return new BooleanEvent(loop, () -> getLeftTriggerAxis() > threshold);
        }

        /**
         * Constructs an event instance around the axis value of the left trigger. The
         * returned trigger
         * will be true when the axis value is greater than 0.5.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance that is true when the left trigger's axis exceeds
         *         the provided
         *         threshold, attached to the given event loop
         */
        public BooleanEvent leftTrigger(EventLoop loop) {
            return leftTrigger(0.5, loop);
        }

        /**
         * Constructs an event instance around the axis value of the right trigger. The
         * returned trigger
         * will be true when the axis value is greater than {@code threshold}.
         *
         * @param threshold the minimum axis value for the returned {@link BooleanEvent}
         *                  to be true. This
         *                  value should be in the range [0, 1] where 0 is the unpressed
         *                  state of the axis.
         * @param loop      the event loop instance to attach the event to.
         * @return an event instance that is true when the right trigger's axis exceeds
         *         the provided
         *         threshold, attached to the given event loop
         */
        public BooleanEvent rightTrigger(double threshold, EventLoop loop) {
            return new BooleanEvent(loop, () -> getRightTriggerAxis() > threshold);
        }

        /**
         * Constructs an event instance around the axis value of the right trigger. The
         * returned trigger
         * will be true when the axis value is greater than 0.5.
         *
         * @param loop the event loop instance to attach the event to.
         * @return an event instance that is true when the right trigger's axis exceeds
         *         the provided
         *         threshold, attached to the given event loop
         */
        public BooleanEvent rightTrigger(EventLoop loop) {
            return rightTrigger(0.5, loop);
        }
    }
}
