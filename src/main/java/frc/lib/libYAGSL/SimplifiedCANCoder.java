// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.libYAGSL;

import static frc.lib.lib2706.ErrorCheck.errCTRE;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.lib.lib6328.Alert;
import frc.robot.Config.GeneralConfig;

/**
 * Class to represent the cancoder on the steering of a swerve module.
 *
 * Copied from YAGSL:
 * Exact file: http://github.com/BroncBotz3481/YAGSL/blob/main/swervelib/cancoders/CANCoderSwerve.java
 * Repo: http://github.com/BroncBotz3481/YAGSL
 */
public class SimplifiedCANCoder {
    private final int maximumRetries = GeneralConfig.ctreMaxRetries;
    private final double statusTimeoutSeconds = 0.02;
    private final double optimizeBusTimeout = 1.0;

    private CANcoder cancoder;
    private StatusSignal<Double> absolutePosition, velocity;

    private String name;
    private Alert cannotConfigure,
            magnetFieldLessThanIdeal,
            readingFaulty,
            readingIgnored,
            cannotSetOffset;

    public SimplifiedCANCoder(int canID, String name) {
        cancoder = new CANcoder(canID, "rio");
        this.name = name;

        absolutePosition = cancoder.getAbsolutePosition();
        velocity = cancoder.getVelocity();

        cannotConfigure =
                new Alert("Cancoders", "Failure to configure " + name, Alert.AlertType.ERROR);
        magnetFieldLessThanIdeal =
                new Alert(
                        "Cancoders",
                        name + " magnetic field is less than ideal.",
                        Alert.AlertType.ERROR);
        readingFaulty =
                new Alert("Cancoders", name + " reading was faulty.", Alert.AlertType.ERROR);
        readingIgnored =
                new Alert(
                        "Cancoders",
                        name + " reading was faulty, ignoring.",
                        Alert.AlertType.ERROR);
        cannotSetOffset =
                new Alert(
                        "Cancoders",
                        "Failure to set " + name + " Absolute Cancoder Offset",
                        Alert.AlertType.ERROR);
    }

    /**
     * Reset the cancoder to factory defaults.
     */
    public void factoryDefault() {
        cancoder.getConfigurator().apply(new CANcoderConfiguration());
    }

    /**
     * Clear sticky faults on the cancoder.
     */
    public void clearStickyFaults() {
        cancoder.clearStickyFaults();
    }

    /**
     * Configure the absolute cancoder to read from [0, 360) per second.
     *
     * @param inverted Whether the cancoder is inverted.
     */
    public void configure(boolean inverted, double statusUpdatePeriod) {
        CANcoderConfigurator cfg = cancoder.getConfigurator();
        MagnetSensorConfigs magnetSensorConfiguration =
                new MagnetSensorConfigs()
                        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                        .withSensorDirection(
                                inverted
                                        ? SensorDirectionValue.Clockwise_Positive
                                        : SensorDirectionValue.CounterClockwise_Positive);

        absolutePosition = cancoder.getAbsolutePosition();
        velocity = cancoder.getVelocity();

        cfg.refresh(magnetSensorConfiguration);

        // Apply configs
        boolean error = false;
        for (int i = 0; i < maximumRetries; i++) {
            error = cfg.apply(magnetSensorConfiguration) == StatusCode.OK;
            if (!error) break;
        }

        cannotConfigure.set(error);

        errCTRE(
                name + " set update freq",
                BaseStatusSignal.setUpdateFrequencyForAll(
                        1.0 / statusUpdatePeriod, absolutePosition, velocity));

        errCTRE(name + " optimize canbus", cancoder.optimizeBusUtilization(optimizeBusTimeout));
    }

    /**
     * Get the absolute position of the cancoder.
     *
     * @return Absolute position in degrees from [0, 360).
     */
    public double getAbsolutePosition() {
        MagnetHealthValue strength = cancoder.getMagnetHealth().getValue();

        magnetFieldLessThanIdeal.set(strength != MagnetHealthValue.Magnet_Green);
        if (strength == MagnetHealthValue.Magnet_Invalid
                || strength == MagnetHealthValue.Magnet_Red) {
            readingFaulty.set(true);
            return 0;
        } else {
            readingFaulty.set(false);
        }

        // Taken from democat's library.
        // Source:
        // https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
        for (int i = 0; i < maximumRetries; i++) {
            if (absolutePosition.getStatus() == StatusCode.OK) {
                break;
            }
            absolutePosition = absolutePosition.waitForUpdate(statusTimeoutSeconds);
        }
        if (absolutePosition.getStatus() != StatusCode.OK) {
            readingIgnored.set(true);
        } else {
            readingIgnored.set(false);
        }

        return absolutePosition.getValue() * 360;
    }

    /**
     * Sets the Absolute Cancoder Offset within the CANcoder's Memory.
     *
     * @param offset the offset the Absolute Cancoder uses as the zero point in degrees.
     * @return if setting Absolute Cancoder Offset was successful or not.
     */
    public boolean setAbsoluteCancoderOffset(double offset) {
        CANcoderConfigurator cfg = cancoder.getConfigurator();
        MagnetSensorConfigs magCfg = new MagnetSensorConfigs();
        StatusCode error = cfg.refresh(magCfg);
        if (error != StatusCode.OK) {
            return false;
        }
        error = cfg.apply(magCfg.withMagnetOffset(offset / 360));
        cannotSetOffset.setText(
                "Failure to set " + name + " Absolute Cancoder Offset Error: " + error);
        if (error == StatusCode.OK) {
            cannotSetOffset.set(false);
            return true;
        }
        cannotSetOffset.set(true);
        return false;
    }

    /**
     * Get the velocity in degrees/sec.
     *
     * @return velocity in degrees/sec.
     */
    public double getVelocity() {
        return velocity.getValue() * 360;
    }
}
