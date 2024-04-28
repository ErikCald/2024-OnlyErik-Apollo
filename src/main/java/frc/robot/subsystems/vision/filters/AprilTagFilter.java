package frc.robot.subsystems.vision.filters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

/**
 * A class to represents a filter for the estimated robot pose based on some criteria.
 *
 * <p> Extend this class and override the filterEstimatedPose method to implement custom filtering logic.
 */
public class AprilTagFilter {
    protected Matrix<N3, N1> m_defaultVisionStdDev;

    /**
     * Creates a new AprilTagFilterInterface object.
     *
     * @param defaultVisionStdDev the default vision standard deviation
     */
    public AprilTagFilter(Matrix<N3, N1> defaultVisionStdDev) {
        m_defaultVisionStdDev = defaultVisionStdDev;
    }

    /**
     * Creates a new AprilTagFilterInterface object.
     * <p>
     * Sets the default vision standard deviation to the default values
     * in SwerveDrivePoseEstimator.
     */
    public AprilTagFilter() {
        m_defaultVisionStdDev = VecBuilder.fill(0.9, 0.9, 0.9);
    }

    /**
     * Updates the default vision standard deviation.
     *
     * @param defaultVisionStdDev the new default vision standard deviation
     */
    public void updateDefaultVisionStdDev(Matrix<N3, N1> defaultVisionStdDev) {
        m_defaultVisionStdDev = defaultVisionStdDev;
    }

    /**
     * Filters the estimated robot pose based on some criteria.
     *
     * @param estimatedRobotPose the estimated pose of the robot
     * @param tagIdsUsed the array of tag IDs used for filtering
     * @return Vision measurement standard deviations for the given data,
     *         or an empty optional if the data should ber rejected
     */
    public Optional<Matrix<N3, N1>> filterEstimatedPose(
            EstimatedRobotPose estimatedRobotPose, long[] tagIdsUsed) {
        return Optional.of(m_defaultVisionStdDev);
    }
}
