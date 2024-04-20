package frc.lib.lib254;

/**
 * Represents the kinematic limits of a swerve drive.
 *
 * This file comes from team 254 and is for SwerveSetpointGenerator.java
 */
public class SwerveModuleLimits {
    public final double kMaxDriveVelocity; // m/s
    public final double kMaxDriveAcceleration; // m/s^2
    public final double kMaxSteeringVelocity; // rad/s

    /**
     * Constructs a new instance of SwerveModuleLimits with the specified limits.
     *
     * @param kMaxDriveVelocity     the maximum drive velocity
     * @param kMaxDriveAcceleration the maximum drive acceleration
     * @param kMaxSteeringVelocity  the maximum steering velocity
     */
    public SwerveModuleLimits(
            double kMaxDriveVelocity, double kMaxDriveAcceleration, double kMaxSteeringVelocity) {
        this.kMaxDriveVelocity = kMaxDriveVelocity;
        this.kMaxDriveAcceleration = kMaxDriveAcceleration;
        this.kMaxSteeringVelocity = kMaxSteeringVelocity;
    }

    /**
     * Returns a string representation of the SwerveModuleLimits object.
     *
     * @return A string representation of the SwerveModuleLimits object.
     */
    @Override
    public String toString() {
        String ret = "Drive Velocity: " + kMaxDriveVelocity + " mps\n";
        ret += "Drive Acceleration: " + kMaxDriveAcceleration + " mpsps\n";
        ret += "Steering Velocity: " + kMaxSteeringVelocity + " radps\n";
        return ret;
    }
}
