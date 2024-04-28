// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Config.PhotonConfig.PhotonCameraConfig;
import frc.robot.subsystems.vision.filters.AprilTagFilter;
import frc.robot.subsystems.vision.filters.FilterForSpeakerShot;

import java.util.ArrayList;

/**
 * The ApriltagSubsystem class represents the subsystem responsible for handling the Photon vision system.
 * It provides a singleton instance and ensures that only one instance of the subsystem is created.
 */
public class ApriltagSubsystem extends SubsystemBase {
    private static ApriltagSubsystem s_instance;
    public static final AprilTagFieldLayout s_fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    private ArrayList<PhotonAprilTagCamera> m_cameras = new ArrayList<>();
    private AprilTagFilter m_apriltagFilter;

    private int m_dataQuality = 0;
    private boolean m_goodData = false;

    private final int BAD_SAMPLE_WORTH = 1;
    private final double DURATION_FOR_STALE_DATA = 8.0;
    private final int SAMPLES_FOR_GOOD_DATA = 20;

    private final int QUALITY_THRESHOLD = (int) (DURATION_FOR_STALE_DATA / 0.02) * BAD_SAMPLE_WORTH;
    private final int GOOD_SAMPLE_WORTH = QUALITY_THRESHOLD / SAMPLES_FOR_GOOD_DATA;

    /**
     * The ApriltagSubsystem class represents the subsystem responsible for handling the Photon vision system.
     * It provides a singleton instance and ensures that only one instance of the subsystem is created.
     */
    public static ApriltagSubsystem getInstance() {
        if (s_instance == null) {
            s_instance = new ApriltagSubsystem();
        }
        return s_instance;
    }

    /**
     * Represents the ApriltagSubsystem, which handles the vision processing for the robot.
     */
    private ApriltagSubsystem() {
        m_apriltagFilter = new FilterForSpeakerShot();

        PhotonAprilTagCamera backApriltag =
                new PhotonAprilTagCamera(
                        PhotonCameraConfig.BACK_APRILTAG.cameraName,
                        PhotonCameraConfig.BACK_APRILTAG.cameraOffset,
                        m_apriltagFilter);
        backApriltag.enableOutputImageSnapshots(2);

        m_cameras.add(backApriltag);
    }

    /**
     * Executes periodic tasks for the ApriltagSubsystem.
     */
    @Override
    public void periodic() {
        // Update the cameras
        boolean successful = false;
        for (PhotonAprilTagCamera camera : m_cameras) {
            successful = camera.update();
        }

        // Update the data quality
        if (successful) {
            m_dataQuality += GOOD_SAMPLE_WORTH;
        } else {
            m_dataQuality -= BAD_SAMPLE_WORTH;
        }
        m_dataQuality = MathUtil.clamp(m_dataQuality, 0, QUALITY_THRESHOLD);

        // Check if the data quality is good
        if (m_dataQuality >= QUALITY_THRESHOLD) {
            m_goodData = true; // Data is good
        } else if (m_dataQuality <= 0) {
            m_goodData = false; // Data is stale
        }
    }

    /**
     * Resets the data quality to 0.
     */
    public void resetDataQuality() {
        m_dataQuality = 0;
        m_goodData = false;
    }

    /**
     * Checks if the ApriltagSubsystem has good data.
     *
     * @return true if the ApriltagSubsystem has good data, false for stale data.
     */
    public boolean hasGoodData() {
        return m_goodData;
    }
}
