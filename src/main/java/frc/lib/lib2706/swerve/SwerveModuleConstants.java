package frc.lib.lib2706.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveCanID, steerCanID, cancoderCanID;
    public final Rotation2d steerOffset;

    /**
     * Represents the constants for a Swerve Module.
     *
     * @param driveCanID The CAN ID of the drive motor.
     * @param steerCanID The CAN ID of the steer motor.
     * @param cancoderCanID The CAN ID of the CANCoder.
     * @param angleOffset The angle offset for the module.
     */
    public SwerveModuleConstants(
            int driveCanID, int steerCanID, int cancoderCanID, Rotation2d steerOffset) {
        this.driveCanID = driveCanID;
        this.steerCanID = steerCanID;
        this.cancoderCanID = cancoderCanID;
        this.steerOffset = steerOffset;
    }
}
