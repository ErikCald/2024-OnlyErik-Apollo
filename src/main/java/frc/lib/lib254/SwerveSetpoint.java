package frc.lib.lib254;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Represents a setpoint for a swerve drive system.
 *
 * This file comes from team 254 and is for SwerveSetpointGenerator.java
 */
public class SwerveSetpoint {
    public ChassisSpeeds chassisSpeeds;
    public SwerveModuleState[] moduleStates;

    /**
     * Constructs a new SwerveSetpoint object with the given chassis speeds and initial module states.
     *
     * @param chassisSpeeds The desired chassis speeds.
     * @param initialStates The initial states of the swerve modules.
     */
    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates) {
        this.chassisSpeeds = chassisSpeeds;
        this.moduleStates = initialStates;
    }

    /**
     * Constructs a new SwerveSetpoint for the initial state of SwerveSetpointGenerator.
     *
     * @param numModules The number of modules in the swerve drive system.
     */
    public SwerveSetpoint(int numModules) {
        this.chassisSpeeds = new ChassisSpeeds();
        this.moduleStates = new SwerveModuleState[numModules];
        for (int i = 0; i < moduleStates.length; ++i) {
            moduleStates[i] = new SwerveModuleState();
        }
    }

    /**
     * Returns a string representation of the SwerveSetpoint object.
     *
     * @return A string representation of the SwerveSetpoint object.
     */
    @Override
    public String toString() {
        String ret = chassisSpeeds.toString() + "\n";
        for (int i = 0; i < moduleStates.length; ++i) {
            ret += "  " + moduleStates[i].toString() + "\n";
        }
        return ret;
    }
}
