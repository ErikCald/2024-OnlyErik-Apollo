// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.lib6328;

import java.util.ArrayList;
import java.util.List;

/**
 * The VirtualSubsystem class is an abstract class that represents a subsystem in a robot.
 * It allows a file to have a periodic() without using up resources that a subsystem normally would.
 *
 * <p> VirtualSubsystem cannot be used as a requirement for a Command.
 *
 * <p> This was written by FRC 6328 Mechanical Advantage.
 */
public abstract class VirtualSubsystem {
    private static List<VirtualSubsystem> subsystems = new ArrayList<>();

    /**
     * Represents a virtual subsystem in the robot code.
     * This class can be extended to create custom virtual subsystems.
     */
    public VirtualSubsystem() {
        subsystems.add(this);
    }

    /**
     * Executes the periodic update for all registered subsystems.
     * This method should be called periodically in the robot's main loop to update all subsystems.
     */
    public static void periodicAll() {
        for (VirtualSubsystem subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    /**
     * This method is called periodically to update the state of the subsystem.
     * Subclasses should override this method to implement the specific behavior of the subsystem.
     */
    public abstract void periodic();
}
