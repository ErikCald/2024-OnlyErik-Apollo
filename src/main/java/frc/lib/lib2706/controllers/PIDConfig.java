package frc.lib.lib2706.controllers;

/**
 * The PIDConfig class represents the configuration parameters for a PID controller.
 * It stores the values for the feedforward gain (kF), proportional gain (kP),
 * integral gain (kI), derivative gain (kD), and integral zone (iZone).
 */
public class PIDConfig {
    /** F */
    public final double kF;

    /** P */
    public final double kP;

    /** I */
    public final double kI;

    /** D */
    public final double kD;

    /** Integral range */
    public final double iZone;

    /** PID slot index */
    public final int pidSlot;

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
    public PIDConfig(double kF, double kP, double kI, double kD, double iZone, int pidSlot) {
        this.kF = kF;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iZone = iZone;
        this.pidSlot = pidSlot;
    }

    /**
     * Create a new PIDConfig for a velocity PID loop.
     *
     * @param kF the feedforward gain
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param iZone the integral zone
     */
    public PIDConfig(double kF, double kP, double kI, double kD, double iZone) {
        this(kF, kP, kI, kD, iZone, 0);
    }

    /**
     * Create a new PIDConfig for a position PID loop.
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param iZone the integral zone
     */
    public PIDConfig(double kP, double kI, double kD, double iZone) {
        this(0.0, kP, kI, kD, iZone, 0);
    }

    /**
     * Create a new PIDConfig for a position PID loop at a given PID slot.
     *
     * @param kP the proportional gain
     * @param kI the integral gain
     * @param kD the derivative gain
     * @param iZone the integral zone
     */
    public PIDConfig(double kP, double kI, double kD, double iZone, int pidSlot) {
        this(0.0, kP, kI, kD, iZone, pidSlot);
    }
}
