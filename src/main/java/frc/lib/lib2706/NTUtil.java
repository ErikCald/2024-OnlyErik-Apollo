// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.lib2706;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;

import frc.robot.Config.NTConfig;

/**
 * Utility class for NetworkTables.
 */
public class NTUtil {
    private static final double SLOW_PERIOD = NTConfig.SLOW_PERIODIC_SECONDS;
    private static final double FAST_PERIOD = NTConfig.FAST_PERIODIC_SECONDS;

    /** Utility class, so constructor is private. */
    private NTUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Creates a DoublePublisher object for publishing double values to a specific topic.
     *
     * @param moduleTable the NetworkTable object representing the table to publish the values to
     * @param topicName the name of the topic to publish the double values to
     * @return a DoublePublisher object for publishing double values
     */
    public static DoublePublisher createDoublePublisherFast(
            NetworkTable moduleTable, String topicName) {
        return moduleTable.getDoubleTopic(topicName).publish(PubSubOption.periodic(FAST_PERIOD));
    }

    /**
     * Creates a DoublePublisher object for publishing double values to a specific topic.
     *
     * @param moduleTable the NetworkTable object representing the table to publish the values to
     * @param topicName the name of the topic to publish the double values to
     * @return a DoublePublisher object for publishing double values
     */
    public static DoublePublisher createDoublePublisherSlow(
            NetworkTable moduleTable, String topicName) {
        return moduleTable.getDoubleTopic(topicName).publish(PubSubOption.periodic(SLOW_PERIOD));
    }
}
