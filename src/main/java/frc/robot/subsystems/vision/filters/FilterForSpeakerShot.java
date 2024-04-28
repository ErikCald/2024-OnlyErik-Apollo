// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.filters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.util.Optional;

/**
 * This class represents a filter for processing vision data related to a speaker shot.
 * It initializes the default vision standard deviation values.
 */
public class FilterForSpeakerShot extends AprilTagFilter {
    /**
     * Filters the estimated robot pose to only allow poses created
     * from more than 1 tag.
     *
     * @param estimatedRobotPose the estimated pose of the robot
     * @param tagIdsUsed the array of tag IDs used for filtering
     * @return Vision measurement standard deviations for the given data,
     *         or an empty optional if the data should ber rejected
     */
    @Override
    public Optional<Matrix<N3, N1>> filterEstimatedPose(
            EstimatedRobotPose estimatedRobotPose, long[] tagIdsUsed) {
        // Must use multi-tag PnP
        if (estimatedRobotPose.strategy != PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                && estimatedRobotPose.strategy != PoseStrategy.MULTI_TAG_PNP_ON_RIO) {
            return Optional.empty();
        }

        // Must have more than 1 tag in frame
        if (estimatedRobotPose.targetsUsed.size() <= 1) {
            return Optional.empty();
        }

        // Must contain close pairs of tags
        if (!containsPairs(tagIdsUsed)) {
            return Optional.empty();
        }

        return Optional.of(m_defaultVisionStdDev);
    }

    /**
     * Checks if the given array of tag IDs contains pairs of specific tag IDs.
     *
     * @param tagIdsUsed the array of tag IDs to check
     * @return true if the array contains pairs of specific tag IDs, false otherwise
     */
    private boolean containsPairs(long[] tagIdsUsed) {
        // Blue source speaker tags
        if (contains(tagIdsUsed, 1) && contains(tagIdsUsed, 2)) {
            return true;
        }

        // Red side speaker tags
        if (contains(tagIdsUsed, 3) && contains(tagIdsUsed, 4)) {
            return true;
        }

        // Blue side speaker tags
        if (contains(tagIdsUsed, 7) && contains(tagIdsUsed, 8)) {
            return true;
        }

        // Red source speaker tags
        if (contains(tagIdsUsed, 9) && contains(tagIdsUsed, 10)) {
            return true;
        }

        return false;
    }

    /**
     * Checks if a given value is present in an array of long integers.
     *
     * @param arr the array of long integers to search in
     * @param val the value to search for
     * @return true if the value is found in the array, false otherwise
     */
    private boolean contains(long[] arr, long val) {
        for (long l : arr) {
            if (l == val) {
                return true;
            }
        }
        return false;
    }
}
