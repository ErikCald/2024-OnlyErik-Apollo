package frc.lib.lib2706.networktables;

import edu.wpi.first.networktables.NetworkTable;

import frc.lib.lib2706.controllers.PIDConfig;
import frc.lib.lib2706.controllers.ProfiledPIDConfig;

import java.util.function.Consumer;

/**
 * A class representing a tunable PID configuration.
 */
public class TunableProfiledPIDConfig {
    private final TunablePIDConfig m_tunablePIDConfig;
    private final TunableDouble m_velConstraint, m_accelConstraint;
    private final Consumer<ProfiledPIDConfig> m_setConfig;

    private PIDConfig m_currentPIDConfig;

    /**
     * Creates a new TunableProfiledPIDConfig with the given PIDConfig, velocity constraint, and acceleration constraint.
     *
     * @param setConfig Consumer to set the PID configuration. Example: {@code (config) -> config.applyConfig(m_pidX)}
     * @param table The NetworkTable to read and write values to
     * @param pidConfig The PIDConfig for the ProfiledPIDController
     * @param velConstraint The velocity constraint
     * @param accelConstraint The acceleration constraint
     */
    public TunableProfiledPIDConfig(
            Consumer<ProfiledPIDConfig> setConfig,
            NetworkTable table,
            PIDConfig pidConfig,
            double velConstraint,
            double accelConstraint) {

        m_setConfig = setConfig;
        m_currentPIDConfig = pidConfig;

        NetworkTable pidTable = table.getSubTable("TuneProfiledPID");
        m_velConstraint = new TunableDouble("VelConstraint", pidTable, velConstraint);
        m_accelConstraint = new TunableDouble("AccelConstraint", pidTable, accelConstraint);

        m_tunablePIDConfig = new TunablePIDConfig(this::pidConfigHasUpdates, pidTable, pidConfig);

        // Set the initial values to the defaults or to previously set values on networktables
        updateValues();
    }

    /**
     * Creates a new TunableProfiledPIDConfig with the given PIDConfig, velocity constraint, and acceleration constraint.
     *
     * @param setConfig Consumer to set the PID configuration. Example: {@code (config) -> config.applyConfig(m_pidX)}
     * @param table The NetworkTable to read and write values to
     * @param pidConfig The ProfiledPIDConfig for the ProfiledPIDController
     */
    public TunableProfiledPIDConfig(
            Consumer<ProfiledPIDConfig> setConfig,
            NetworkTable table,
            ProfiledPIDConfig pidConfig) {
        this(
                setConfig,
                table,
                pidConfig.kPIDConfig,
                pidConfig.kVelConstraint,
                pidConfig.kAccelConstraint);
    }

    private void pidConfigHasUpdates(PIDConfig newConfig) {
        m_currentPIDConfig = newConfig;
        updateValues();
    }

    /**
     * Updates the PID controller with the current values of the tunable parameters.
     */
    public void updateValues() {
        m_setConfig.accept(
                new ProfiledPIDConfig(
                        m_currentPIDConfig, m_velConstraint.get(), m_accelConstraint.get()));
    }

    /**
     * Checks for updates in the tunable parameters and updates the feedforward values accordingly.
     */
    public void checkForUpdates() {
        m_tunablePIDConfig.checkForUpdates();
        TunableDouble.ifChanged(
                hashCode(), () -> this.updateValues(), m_velConstraint, m_accelConstraint);
    }
}
