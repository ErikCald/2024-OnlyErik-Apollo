// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.lib2706;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;

import frc.robot.Config;
import frc.robot.Config.NTConfig;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/**
 * Class for a tunable double. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableDouble implements DoubleSupplier {
    private static final boolean IN_TUNING_MODE = Config.tuningMode;
    private static final double ENTRY_PERIOD = NTConfig.SLOW_PERIODIC_SECONDS;
    private final DoubleEntry entry;
    private double defaultValue;
    private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    /**
     * Create a new LoggedTunableDouble with the default value
     *
     * @param topicName Name of the topic
     * @param table NetworkTable to use
     * @param defaultValue Default value
     */
    public TunableDouble(String topicName, NetworkTable table, double defaultValue) {
        entry =
                table.getDoubleTopic(topicName)
                        .getEntry(defaultValue, PubSubOption.periodic(ENTRY_PERIOD));
        initDefault(defaultValue);
    }

    /**
     * Create a new LoggedTunableDouble with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public TunableDouble(DoubleTopic topic, double defaultValue) {
        entry = topic.getEntry(defaultValue);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the number. The default value can only be set once.
     *
     * @param defaultValue The default value
     */
    public void initDefault(double defaultValue) {
        entry.setDefault(defaultValue);
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public double get() {
        if (IN_TUNING_MODE) {
            return entry.getAsDouble();
        } else {
            return defaultValue;
        }
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false
     *     otherwise.
     */
    public boolean hasChanged(int id) {
        if (!IN_TUNING_MODE) return false;
        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }

    /**
     * Runs action if any of the TuneableDoubles have changed
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable numbers have changed. Access tunable
     *     numbers in order inputted in method
     * @param tunableNumbers All tunable numbers to check
     */
    public static void ifChanged(
            int id, Consumer<double[]> action, TunableDouble... tunableNumbers) {
        if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
            action.accept(Arrays.stream(tunableNumbers).mapToDouble(TunableDouble::get).toArray());
        }
    }

    /** Runs action if any of the tunableNumbers have changed */
    public static void ifChanged(int id, Runnable action, TunableDouble... tunableNumbers) {
        ifChanged(id, values -> action.run(), tunableNumbers);
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}
