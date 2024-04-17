package frc.lib.lib2706;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The GeomUtil class provides utility methods for geometric calculations.
 */
public class GeomUtil {
    /** Utility class, so constructor is private. */
    private GeomUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Rotates a given angle based on the alliance color.
     * If the alliance color is blue, the angle remains unchanged.
     * If the alliance color is red, the angle is rotated by 180 degrees.
     *
     * @param angle The angle to rotate.
     * @return The rotated angle.
     */
    public static Rotation2d rotateForAlliance(Rotation2d angle) {
        var alliance = DriverStation.getAlliance();

        // Default to blue alliance
        if (alliance.isEmpty()) {
            DriverStation.reportWarning("Unable to detect alliance color.", false);
            return angle;
        }

        if (alliance.get() == DriverStation.Alliance.Blue) {
            return angle;
        } else {
            return angle.rotateBy(new Rotation2d(Math.PI));
        }
    }
}
