package frc.robot.controller;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class FlySkyController {
  public double getRawAxis(int axis) {
    return getRawAxisMap.get(axis).getAsDouble();
  }

  public boolean getRawButton(int button) {
    return getRawBottonMap.get(button).getAsBoolean();
  }

  private final Map<Integer, DoubleSupplier> getRawAxisMap = Map.of(
    Axis.kLeftX.value, this::getLeftX,
    Axis.kLeftY.value, this::getLeftY,
    Axis.kRightX.value, this::getRightX,
    Axis.kRightY.value, this::getRightY
  );

  private final Map<Integer, BooleanSupplier> getRawBottonMap = Map.of(
    Button.kLeftSwitch.value, this::getLeftSwitch,
    Button.kRightSwitch.value, this::getRightSwitch
  );

  public enum Button {
    /** Leftmost switch */
    kLeftSwitch(8),
    /** Second Leftmost switch. */
    kRightSwitch(9);

    /** Button value. */
    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and appending `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

  public enum Axis {
    /** Left X axis. */
    kLeftX(4),
    /** Right X axis. */
    kRightX(7),
    /** Left Y axis. */
    kLeftY(5),
    /** Right Y axis. */
    kRightY(6);

    /** Axis value. */
    public final int value;

    Axis(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the axis, matching the relevant methods. This
     * is done by
     * stripping the leading `k`, and appending `Axis` if the name ends with
     * `Trigger`.
     *
     * <p>
     * Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the axis.
     */
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }

  private final PWMInput leftX;
  private final PWMInput leftY;
  private final PWMInput rightX;
  private final PWMInput rightY;
  private final PWMInput leftSwitch;
  private final PWMInput rightSwitch;

  private boolean lastLeftSwitch1Value = false;
  private boolean lastLeftSwitch2Value = false;

  public FlySkyController(
    int leftXPort, int leftYPort, int rightXPort, int rightYPort, int leftSwitchPort, int rightSwitchPort
  ) {
    leftX = new PWMInput(leftXPort);
    leftY = new PWMInput(leftYPort);
    rightX = new PWMInput(rightXPort);
    rightY = new PWMInput(rightYPort);
    leftSwitch = new PWMInput(leftSwitchPort);
    rightSwitch = new PWMInput(rightSwitchPort);
  }

  public FlySkyController() {
    leftX = new PWMInput(Axis.kLeftX.value);
    leftY = new PWMInput(Axis.kLeftY.value);
    rightX = new PWMInput(Axis.kRightX.value);
    rightY = new PWMInput(Axis.kRightY.value);
    leftSwitch = new PWMInput(Button.kLeftSwitch.value);
    rightSwitch = new PWMInput(Button.kRightSwitch.value);
  }

  int counter = 0;
  
  /**
   * Read the value of the leftmost switch on the controller.
   *
   * @return The state of the switch.
   */
  public boolean getLeftSwitch() {
    lastLeftSwitch1Value = leftSwitch.get() > 0;
    return lastLeftSwitch1Value;
  }

  /**
   * Whether the leftmost switch was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftSwitch1Pressed() {
    boolean result = !lastLeftSwitch1Value && getLeftSwitch();
    lastLeftSwitch1Value = getLeftSwitch();
    return result;
  }

  /**
   * Whether the leftmost switch was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftSwitch1Released() {
    boolean result = lastLeftSwitch1Value && !getLeftSwitch();
    lastLeftSwitch1Value = getLeftSwitch();
    return result;
  }

    /**
   * Read the value of the second leftmost switch on the controller.
   *
   * @return The state of the switch.
   */
  public boolean getRightSwitch() {
    lastLeftSwitch2Value = rightSwitch.get() > 0;
    return lastLeftSwitch2Value;
  }

  /**
   * Whether the second leftmost switch was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getLeftSwitch2Pressed() {
    boolean result = !lastLeftSwitch2Value && getRightSwitch();
    lastLeftSwitch2Value = getRightSwitch();
    return result;
  }

  /**
   * Whether the second leftmost was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getLeftSwitch2Released() {
    boolean result = lastLeftSwitch2Value && !getRightSwitch();
    lastLeftSwitch2Value = getRightSwitch();
    return result;
  }

  /**
   * Get the X axis value of left side of the controller. Right is positive.
   *
   * @return The axis value.
   */
  public double getLeftX() {
    return leftX.get();
  }

  /**
   * Get the X axis value of right side of the controller. Right is positive.
   *
   * @return The axis value.
   */
  public double getRightX() {
    return rightX.get();
  }

  /**
   * Get the Y axis value of left side of the controller. Back is positive.
   *
   * @return The axis value.
   */
  public double getLeftY() {
    return leftY.get();
  }

  /**
   * Get the Y axis value of right side of the controller. Back is positive.
   *
   * @return The axis value.
   */
  public double getRightY() {
    return rightY.get();
  }
}
