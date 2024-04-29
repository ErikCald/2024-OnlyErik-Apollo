package frc.lib.lib2706.controllers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The PIDConfig class represents the configuration parameters for a PID controller.
 * It stores the values for the feedforward gain (kF), proportional gain (kP),
 * integral gain (kI), derivative gain (kD), and integral zone (iZone).
 */
public class ProfiledPIDConfig {
    /** Velocity constraint */
    public final double kVelConstraint;

    /** Acceleration constraint */
    public final double kAccelConstraint;

    /** Config for PID controller */
    public final PIDConfig kPIDConfig;

    /**
     * Create a new PIDConfig for a velocity PID loop at a given PID slot.
     *
     * @param kF the feedforward gain
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param iZone the integral zone
     * @param pidSlot the PID slot index
     */

    /**
     * Create a new ProfiledPIDConfig with the given PIDConfig, velocity constraint, and acceleration constraint.
     *
     * @param pidConfig The PIDConfig for the PID controller
     * @param velConstraint The velocity constraint
     * @param accelConstraint The acceleration constraint
     */
    public ProfiledPIDConfig(PIDConfig pid, double vel, double accel) {
        kPIDConfig = pid;
        kVelConstraint = vel;
        kAccelConstraint = accel;
    }

    /**
     * Creates a new instance of ProfiledPIDController with the specified PID constants and constraints.
     *
     * @return a new instance of ProfiledPIDController
     */
    public ProfiledPIDController createController() {
        return new ProfiledPIDController(
                kPIDConfig.kP,
                kPIDConfig.kI,
                kPIDConfig.kD,
                new Constraints(kVelConstraint, kAccelConstraint));
    }

    /**
     * Creates a new instance of ProfiledExternalPIDController.
     *
     * @return a new instance of ProfiledExternalPIDController
     */
    public ProfiledExternalPIDController createExternalController() {
        return new ProfiledExternalPIDController(new Constraints(kVelConstraint, kAccelConstraint));
    }

    /**
     * Applies the configuration settings to the given ProfiledPIDController.
     *
     * @param controller The ProfiledPIDController to apply the configuration to.
     */
    public void applyConfig(ProfiledPIDController controller) {
        controller.setConstraints(new Constraints(kVelConstraint, kAccelConstraint));
        controller.setPID(kPIDConfig.kP, kPIDConfig.kI, kPIDConfig.kD);
        controller.setIZone(kPIDConfig.iZone);
    }

    /**
     * Applies the configuration to the given ProfiledExternalPIDController.
     * Sets the velocity and acceleration constraints of the controller.
     *
     * @param controller the ProfiledExternalPIDController to apply the configuration to.
     */
    public void applyConfig(ProfiledExternalPIDController controller) {
        controller.setConstraints(new Constraints(kVelConstraint, kAccelConstraint));
    }
}
