// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.lib2706.networktables;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;

import frc.robot.Config.GeneralConfig;
import frc.robot.Config.NTConfig;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Class for a tunable String. Gets value from dashboard in tuning mode, or returns default
 * if not in tuning mode or the value is not in the dashboard.
 */
public class TunableString implements Supplier<String> {
    private static final boolean IN_TUNING_MODE = GeneralConfig.enableTunableData;
    private static final double ENTRY_PERIOD = NTConfig.SLOW_PERIODIC_SECONDS;
    private final StringEntry entry;
    private String defaultValue;
    private Map<Integer, String> lastHasChangedValues = new HashMap<>();

    /**
     * Create a new TunableString with the default value
     *
     * @param topicName Name of the topic
     * @param table NetworkTable to use
     * @param defaultValue Default value
     */
    public TunableString(String topicName, NetworkTable table, String defaultValue) {
        entry =
                table.getStringTopic(topicName)
                        .getEntry(defaultValue, PubSubOption.periodic(ENTRY_PERIOD));
        initDefault(defaultValue);
    }

    /**
     * Create a new TunableString with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public TunableString(StringTopic topic, String defaultValue) {
        entry = topic.getEntry(defaultValue);
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the String.
     * Will not overwrite any value set on the dashboard.
     *
     * @param defaultValue The default value
     */
    public void initDefault(String defaultValue) {
        entry.setDefault(defaultValue);
        this.defaultValue = defaultValue;
    }

    /**
     * Get the current value. Will be from the dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    @Override
    public String get() {
        if (IN_TUNING_MODE) {
            return entry.get();
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
        String currentValue = get();
        String lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || !currentValue.equals(lastValue)) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }

        return false;
    }

    /**
     * Runs action if any of the TunableStrings have changed
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *     objects. Recommended approach is to pass the result of "hashCode()"
     * @param action Callback to run when any of the tunable numbers have changed. Access tunable
     *     numbers in order inputted in method
     * @param tunableNumbers All tunable numbers to check
     */
    public static void ifChanged(
            int id, Consumer<String[]> action, TunableString... tunableStrings) {
        if (Arrays.stream(tunableStrings).anyMatch(tunableString -> tunableString.hasChanged(id))) {
            action.accept(
                    Arrays.stream(tunableStrings).map(TunableString::get).toArray(String[]::new));
        }
    }

    /**
     * Executes the specified action if any of the provided tunable Strings have changed.
     *
     * @param id The identifier for the action.
     * @param action The action to be executed if any of the tunable numbers have changed.
     * @param tunableStrings The tunable Strings to be checked for changes.
     */
    public static void ifChanged(int id, Runnable action, TunableString... tunableStrings) {
        ifChanged(id, values -> action.run(), tunableStrings);
    }
}
