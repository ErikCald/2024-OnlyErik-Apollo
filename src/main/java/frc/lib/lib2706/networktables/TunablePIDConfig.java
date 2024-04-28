package frc.lib.lib2706.networktables;

import edu.wpi.first.networktables.NetworkTable;

import frc.lib.lib2706.controllers.PIDConfig;

import java.util.function.Consumer;

/**
 * A class representing a tunable PID configuration.
 */
public class TunablePIDConfig {
    private final TunableDouble m_kF, m_kP, m_kI, m_kD, m_kIZone;
    private final Consumer<PIDConfig> m_setConfig;
    private final int m_pidSlot;

    /**
     * Constructs a TunablePIDConfig object.
     *
     * @param setConfig         The consumer function to set the PID configuration.
     * @param table             The NetworkTable used for tuning the PID gains and feedforward values.
     * @param kF                The feedforward gain.
     * @param kP                The proportional gain.
     * @param kI                The integral gain.
     * @param kD                The derivative gain.
     * @param kIZone            The integral zone.
     */
    public TunablePIDConfig(
            Consumer<PIDConfig> setConfig,
            NetworkTable table,
            double kF,
            double kP,
            double kI,
            double kD,
            double kIZone,
            int pidSlot) {

        m_setConfig = setConfig;
        m_pidSlot = pidSlot;

        NetworkTable ffTable = table.getSubTable("TunePID");
        m_kF = new TunableDouble("F", ffTable, kF);
        m_kP = new TunableDouble("P", ffTable, kP);
        m_kI = new TunableDouble("I", ffTable, kI);
        m_kD = new TunableDouble("D", ffTable, kD);
        m_kIZone = new TunableDouble("iZone", ffTable, kIZone);

        // Set the initial values to the defaults or to previously set values on networktables
        updateValues();
    }

    /**
     * Constructs a TunablePIDConfig object.
     *
     * @param setConfig The consumer function to set the PID configuration.
     * @param table     The NetworkTable used for tuning the PID gains and feedforward values.
     * @param pidConfig The PIDConfig object to use for the initial values.
     */
    public TunablePIDConfig(
            Consumer<PIDConfig> setConfig, NetworkTable table, PIDConfig pidConfig) {
        this(
                setConfig,
                table,
                pidConfig.kF,
                pidConfig.kP,
                pidConfig.kI,
                pidConfig.kD,
                pidConfig.iZone,
                pidConfig.pidSlot);
    }

    /**
     * Updates the PID controller with the current values of the tunable parameters.
     */
    private void updateValues() {
        m_setConfig.accept(
                new PIDConfig(
                        m_kF.get(), m_kP.get(), m_kI.get(), m_kD.get(), m_kIZone.get(), m_pidSlot));
    }

    /**
     * Checks for updates in the tunable parameters and updates the feedforward values accordingly.
     */
    public void checkForUpdates() {
        TunableDouble.ifChanged(hashCode(), () -> this.updateValues(), m_kP, m_kI, m_kD, m_kIZone);
    }
}
