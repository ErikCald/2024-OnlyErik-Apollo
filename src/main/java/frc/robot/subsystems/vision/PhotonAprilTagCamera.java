// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;

import frc.lib.lib2706.networktables.AdvantageUtil;
import frc.lib.lib2706.networktables.NTUtil;
import frc.robot.Config.NTConfig;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.filters.AprilTagFilter;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Optional;

/**
 * Represents a Photon AprilTag camera used for vision processing.
 * This class provides methods to initialize the camera, estimate pose, and publish data to NetworkTables.
 */
public class PhotonAprilTagCamera {
    private PhotonCamera m_camera;
    private PhotonPoseEstimator m_estimator;
    private AprilTagFilter m_filter;
    private boolean m_takeInputImages = false;
    private boolean m_takeOutputImages = false;
    private double m_imagePeriod = 0.0;
    private final double m_minimumImagePeriod = 0.45;
    private Timer m_imageTimer = new Timer();

    private DoubleArrayPublisher pubEstPose, pubTagPoses, pubStdDev;
    private IntegerArrayPublisher pubTagIds;
    private StringPublisher pubDebugMsg;

    /**
     * Represents a Photon AprilTag camera used for vision processing.
     * This class provides methods to initialize the camera, estimate pose, and publish data to NetworkTables.
     *
     * @param cameraName The name of the camera.
     * @param robotToCamera The transformation matrix representing the pose of the camera relative to the robot.
     */
    public PhotonAprilTagCamera(
            String cameraName, Transform3d robotToCamera, AprilTagFilter filter) {
        m_camera = new PhotonCamera(cameraName);
        m_filter = filter;
        m_imageTimer.start();
        m_estimator =
                new PhotonPoseEstimator(
                        ApriltagSubsystem.s_fieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        m_camera,
                        robotToCamera);

        NetworkTable table = NTConfig.visionTable.getSubTable(cameraName);
        pubEstPose = NTUtil.doubleArrayPubFast(table, "estimatedPose");
        pubTagPoses = NTUtil.doubleArrayPubFast(table, "detectedTagPoses");
        pubStdDev = NTUtil.doubleArrayPubFast(table, "stdDev");
        pubTagIds = table.getIntegerArrayTopic("detectedTagIds").publish(NTUtil.fastPeriodic());
        pubDebugMsg = table.getStringTopic("DebuggingMessage").publish(NTUtil.fastPeriodic());
    }

    /**
     * Updates the PhotonAprilTagCamera by processing the latest camera result and estimating the robot pose.
     * <p>
     * If no new targets are detected, it publishes empty arrays for tag IDs, tag poses, and estimated pose.
     * If targets are detected, it creates arrays of the detected tag IDs and tag poses, and publishes them.
     * It then applies filters to the estimated pose and publishes the filtered pose if it passes all filters.
     * Finally, it logs the estimated pose and sends the measurement to the SwerveSubsystem.
     *
     * @return True if the data was successfully passed to the SwerveSubsystem, false otherwise.
     */
    public boolean update() {
        PhotonPipelineResult cameraResult = m_camera.getLatestResult();
        Optional<EstimatedRobotPose> estimate = m_estimator.update(cameraResult);

        // Check if an no new data is available
        if (estimate.isEmpty()) {
            pubDebugMsg.set("No new targets detected");
            pubTagIds.accept(new long[0]);
            pubTagPoses.accept(new double[0]);
            pubEstPose.accept(new double[0]);
            pubStdDev.accept(new double[0]);
            return false;
        }

        // Capture image snapshots if enabled
        if ((m_takeInputImages || m_takeOutputImages) && m_imageTimer.hasElapsed(m_imagePeriod)) {
            m_imageTimer.reset();

            if (m_takeInputImages) {
                m_camera.takeInputSnapshot();
            } else if (m_takeOutputImages) {
                m_camera.takeOutputSnapshot();
            }
        }

        // Create an array of the detected tag IDs and array of tag poses
        long[] detectedTagIds = new long[estimate.get().targetsUsed.size()];
        ArrayList<Pose3d> detectedTagPoses = new ArrayList<>();
        for (int i = 0; i < estimate.get().targetsUsed.size(); i++) {
            PhotonTrackedTarget target = estimate.get().targetsUsed.get(i);
            detectedTagIds[i] = target.getFiducialId();

            Optional<Pose3d> tagPose =
                    ApriltagSubsystem.s_fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                detectedTagPoses.add(tagPose.get());
            }
        }
        pubTagIds.accept(detectedTagIds);
        pubTagPoses.accept(AdvantageUtil.deconstructPose3ds(detectedTagPoses));

        // Filter the estimated pose based on the filters
        Optional<Matrix<N3, N1>> stdDev =
                m_filter.filterEstimatedPose(estimate.get(), detectedTagIds);
        if (stdDev.isEmpty()) {
            pubDebugMsg.accept("Rejected data by filter called " + m_filter.getClass().getName());
            pubEstPose.accept(new double[0]);
            pubStdDev.accept(new double[0]);
            return false;
        }

        // Log the estimated pose
        pubEstPose.accept(AdvantageUtil.deconstruct(estimate.get().estimatedPose));
        pubStdDev.accept(stdDev.get().getData());
        pubDebugMsg.accept("Accepted data");

        // Send measurement to the swerve subsystem
        SwerveSubsystem.getInstance()
                .addVisionMeasurement(
                        estimate.get().estimatedPose.toPose2d(),
                        estimate.get().timestampSeconds,
                        stdDev.get());
        return true;
    }

    /**
     * Enables capturing input image snapshots at a specified periodic rate.
     * Disables output image snapshots.
     *
     * @param periodicRate The rate at which input image snapshots should be captured.
     */
    public void enableInputImageSnapshots(double periodicRate) {
        m_imagePeriod = Math.min(m_minimumImagePeriod, periodicRate);
        m_takeInputImages = true;
        m_takeOutputImages = false;
    }

    /**
     * Enables the output image snapshots with the specified periodic rate.
     * Disables input image snapshots.
     *
     * @param periodicRate the rate at which the output image snapshots should be taken
     */
    public void enableOutputImageSnapshots(double periodicRate) {
        m_imagePeriod = Math.min(m_minimumImagePeriod, periodicRate);
        m_takeInputImages = false;
        m_takeOutputImages = true;
    }

    /**
     * Disables the capturing of image snapshots.
     */
    public void disableImageSnapshots() {
        m_imagePeriod = Double.MAX_VALUE;
        m_takeInputImages = false;
        m_takeOutputImages = false;
    }
}
