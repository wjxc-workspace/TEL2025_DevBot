package frc.robot.controller;


import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A version of {@link FlySkyController} with {@link Trigger} factories for command-based.
 *
 * @see FlySkyController
 */
@SuppressWarnings("MethodName")
public class CommandFlySkyController {
  private final FlySkyController m_hid;
  private final Map<EventLoop, Map<Integer, Trigger>> m_buttonCache = new HashMap<>();

  /**
   * Construct an instance of a controller.
   */
  public CommandFlySkyController() {
    m_hid = new FlySkyController();
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandFlySkyController(int leftXPort, int leftYPort, int rightXPort, int rightYPort, int leftSwitchPort, int rightSwitchPort) {
    m_hid = new FlySkyController(leftXPort, leftYPort, rightXPort, rightYPort, leftSwitchPort, rightSwitchPort);
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  public FlySkyController getHID() {
    return m_hid;
  }

    /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param button the button index
   * @return an event instance representing the button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #button(int, EventLoop)
   */
  public Trigger button(int button) {
    return button(button, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param button the button index
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the button's digital signal attached to the given loop.
   */
  public Trigger button(int button, EventLoop loop) {
    var cache = m_buttonCache.computeIfAbsent(loop, k -> new HashMap<>());
    return cache.computeIfAbsent(button, k -> new Trigger(loop, () -> m_hid.getRawButton(k)));
  }

  /**
   * Constructs a Trigger instance around the A button's digital signal.
   *
   * @return a Trigger instance representing the A button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #leftSwitch(EventLoop)
   */
  public Trigger leftSwitch() {
    return leftSwitch(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the A button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the A button's digital signal attached
   *     to the given loop.
   */
  public Trigger leftSwitch(EventLoop loop) {
    return button(FlySkyController.Button.kLeftSwitch.value, loop);
  }

  /**
   * Constructs a Trigger instance around the B button's digital signal.
   *
   * @return a Trigger instance representing the B button's digital signal attached
   *     to the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #rightSwitch(EventLoop)
   */
  public Trigger rightSwitch() {
    return rightSwitch(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the B button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the B button's digital signal attached
   *     to the given loop.
   */
  public Trigger rightSwitch(EventLoop loop) {
    return button(FlySkyController.Button.kRightSwitch.value, loop);
  }

  /**
   * Get the X axis value of left side of the controller. Right is positive.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    return m_hid.getLeftX();
  }

  /**
   * Get the X axis value of right side of the controller. Right is positive.
   *
   * @return The axis value.
   */
  public double getRightX() {
    return m_hid.getRightX();
  }

  /**
   * Get the Y axis value of left side of the controller. Back is positive.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    return m_hid.getLeftY();
  }

  /**
   * Get the Y axis value of right side of the controller. Back is positive.
   *
   * @return The axis value.
   */
  public double getRightY() {
    return m_hid.getRightY();
  }
}
