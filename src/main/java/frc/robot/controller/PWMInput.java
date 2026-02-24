package frc.robot.controller;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

public class PWMInput {
    private final Counter counter;

    public PWMInput(int channel) {
        // there is not PWM input on Roborio, so we use DigitalInput + Counter to measure pulse width
        DigitalInput input = new DigitalInput(channel);
        counter = new Counter(input);
        counter.setSemiPeriodMode(true); // only get HIGH not HIGH + LOW m3
        counter.setMaxPeriod(0.1); // 100ms timeout(if there is no rising edge in 100ms, counter.getStopped = true)
    }

    /**
     * @return pulse width
     */
    public double getPulseWidthMicros() {
        if (counter.get() == 0) return -1; // no input yet
        return counter.getPeriod() * 1_000_000; // seconds to μs(microseconds)
    }
    /**
     * @return value after normalized (-1.0 ~ 1.0)
     */
    public double get() {
        double normalized = (getPulseWidthMicros() - 1500) / 500;
        return Math.max(-1, Math.min(1, normalized));
    }
}
