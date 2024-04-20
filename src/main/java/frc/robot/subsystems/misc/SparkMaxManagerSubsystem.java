// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.misc;

import static frc.lib.lib2706.ErrorCheck.errSpark;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.lib6328.Alert;
import frc.lib.lib6328.Alert.AlertType;
import frc.robot.Config.CANID;

import java.util.ArrayList;
import java.util.Map;

/**
 * The SparkMaxManagerSubsystem class represents a subsystem that manages multiple CANSparkMax objects.
 * It provides methods to register CANSparkMax motors and track their errors.
 *
 * This will only check 1 sparkmax each robot cycle to avoid spamming the can bus all at once
 */
public class SparkMaxManagerSubsystem extends SubsystemBase {
    private static SparkMaxManagerSubsystem instance;
    private final ArrayList<SparkMaxManager> m_sparkmaxManagers = new ArrayList<>();
    private final NetworkTable errorTable;
    private int currentSparkmaxIndex = 0;

    public static SparkMaxManagerSubsystem getInstance() {
        if (instance == null) {
            instance = new SparkMaxManagerSubsystem();
        }
        return instance;
    }

    /**
     * The SparkMaxManagerSubsystem class represents a subsystem that manages Spark MAX motor controllers.
     * It provides functionality to handle errors and send them to NetworkTables.
     */
    public SparkMaxManagerSubsystem() {
        errorTable =
                NetworkTableInstance.getDefault()
                        .getTable("CANSparkMax/Errors"); // Errors will be sent to NetworkTables
    }

    /**
     * Run burnFlash() for all controllers initialized. The ideal use case for this call is to call it
     * once everything has been initialized. The burnFlash() call has the side effect of preventing
     * all communication *to* the device for up to 200ms or more, potentially including some messages
     * called before the burnFlash() call, and receiveing messages *from* the device.
     *
     * <p>WARNING: This call will sleep the thread before and after burning flash. This is for your
     * safety.
     */
    public void burnFlashOnAllSparkmaxes() {
        if (m_sparkmaxManagers.isEmpty()) {
            DriverStation.reportWarning(
                    "Cannot burn flash because no SparkMaxs are registered to"
                            + " SparkMaxManagerSubsystem.",
                    false);
            return;
        }

        String sparkmaxs = "";
        for (SparkMaxManager manager : m_sparkmaxManagers) {
            sparkmaxs += manager.m_name + ", ";
        }
        sparkmaxs = sparkmaxs.substring(0, sparkmaxs.length() - 2);

        DriverStation.reportWarning("Burning Flash on all SparkMaxs: " + sparkmaxs, false);
        Timer.delay(0.5); // Delay to ensure all can settings have completed
        for (SparkMaxManager manager : m_sparkmaxManagers) {
            errSpark(manager.m_name + " burn flash", manager.m_sparkMax.burnFlash());
            Timer.delay(0.005); // Delay slightly to avoid spamming the CAN bus
        }
        Timer.delay(0.25); // Delay to ensure burn flash is completed before trying other operations
        DriverStation.reportWarning("Burn Flash Complete", false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (!m_sparkmaxManagers.isEmpty()) { // Ensure motor list is not empty!
            m_sparkmaxManagers.get(currentSparkmaxIndex).update();

            if (currentSparkmaxIndex < m_sparkmaxManagers.size() - 1) {
                currentSparkmaxIndex++;
            } else {
                currentSparkmaxIndex = 0;
            }
        }
    }

    /**
     * Registers the given CANSparkMax motors to be tracked for errors.
     *
     * @param motors the CANSparkMax motors to be registered
     */
    public void register(String alertGroup, CANSparkMax... motors) {
        for (CANSparkMax motor : motors) {
            m_sparkmaxManagers.add(new SparkMaxManager(motor, alertGroup));
        }
    }

    /**
     * Converts a short value representing fault bits into an
     * ArrayList of CANSparkBase.FaultID objects.
     *
     * @param faults the short value representing fault bits
     * @return an array of FaultID objects representing the fault conditions
     */
    private ArrayList<FaultID> faultBitsToFaultID(short faults) {
        ArrayList<FaultID> faultStrings = new ArrayList<>();
        for (int i = 0; i <= 11; i++) {
            if (((1 << i) & (int) faults) != 0) {
                faultStrings.add(CANSparkMax.FaultID.fromId(i));
            }
        }

        return faultStrings;
    }

    /**
     * Convert an array of faults to an array of Strings.
     *
     * @param faults the short value representing fault bits
     * @return the string representation of the fault bits
     */
    private String[] faultArrayToStringArray(ArrayList<FaultID> faults) {
        String[] faultStrings = new String[faults.size()];
        for (int i = 0; i < faults.size(); i++) {
            faultStrings[i] = faults.get(i).name();
        }
        return faultStrings;
    }

    /**
     * Converts an ArrayList of FaultID objects to a single string representation.
     *
     * @param faults the ArrayList of FaultID objects to convert
     * @return a single string representation of the faults
     */
    private String faultArrayToSingleString(ArrayList<FaultID> faults) {
        String faultString = "";
        for (FaultID fault : faults) {
            faultString += fault.name() + ", ";
        }
        return faultString.substring(0, faultString.length() - 2);
    }

    /**
     * The SparkMaxManager class is responsible for managing a CANSparkMax motor controller.
     * It provides functionality to handle faults, update fault status, and raise alerts for major and minor issues.
     */
    private class SparkMaxManager {
        private final CANSparkMax m_sparkMax;
        private final StringArrayPublisher pubErrors;
        private final Alert minorAlert, majorAlert;
        private final String m_name;
        private int m_resetCount = 0;

        /**
         * Constructs a new SparkMaxManager object.
         *
         * @param sparkMax The CANSparkMax motor controller to manage.
         */
        private SparkMaxManager(CANSparkMax sparkMax, String alertGroup) {
            m_sparkMax = sparkMax;

            // Set the name from the map or default to "SparkMax" + canId
            Map<Integer, String> mapCanIdsToNames = CANID.mapCanIdsToNames();
            if (mapCanIdsToNames.containsKey(sparkMax.getDeviceId())) {
                m_name = mapCanIdsToNames.get(sparkMax.getDeviceId()) + sparkMax.getDeviceId();
            } else {
                m_name = "SparkMax" + sparkMax.getDeviceId();
            }

            pubErrors = errorTable.getStringArrayTopic(m_name).publish();

            // Message is updated later based on the faults
            minorAlert = new Alert(alertGroup, m_name, AlertType.WARNING);
            majorAlert = new Alert(alertGroup, m_name, AlertType.ERROR);
        }

        private void update() {
            // Get the sticky faults since the last time this was called and clear faults
            short faultBits = m_sparkMax.getStickyFaults();
            m_sparkMax.clearFaults();

            // Hard code faultBits to test it in simulation. Comment out in real robot
            // faultBits = (short) (1 << (int) FaultID.kDRVFault.value);

            // Publish the faults as an array of strings
            ArrayList<FaultID> faults = faultBitsToFaultID(faultBits);
            pubErrors.set(faultArrayToStringArray(faults));

            // Check for major issues
            boolean hasReset = faults.remove(FaultID.kHasReset);
            boolean hasGateDriverFault = faults.remove(FaultID.kDRVFault);
            boolean hasCanFault = faults.remove(FaultID.kCANTX) || faults.remove(FaultID.kCANRX);
            faults.remove(FaultID.kCANRX);
            boolean hasMajorIssue = hasReset || hasGateDriverFault || hasCanFault;
            String majorFaults = "";
            if (hasGateDriverFault) majorFaults += "gate driver fault (kDRVFault), ";
            if (hasCanFault) majorFaults += "can fault, ";
            if (hasReset) {
                m_resetCount++;
                majorFaults += "reset " + m_resetCount + " times, ";
            }
            if (hasMajorIssue) {
                majorFaults = majorFaults.substring(0, majorFaults.length() - 2);
                majorAlert.setText(m_name + " has major faults: " + majorFaults);
                majorAlert.set(true);
            } else {
                majorAlert.set(false);
            }

            // Check for minor issues
            if (!hasMajorIssue && !faults.isEmpty()) {
                String minorFaults = faultArrayToSingleString(faults);
                minorAlert.setText(m_name + " has minor faults: " + minorFaults);
                minorAlert.set(true);
            } else {
                minorAlert.set(false);
            }
        }
    }
}
