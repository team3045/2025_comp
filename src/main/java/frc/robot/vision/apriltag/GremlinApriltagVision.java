// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.apriltag;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonTargetSortMode;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import static frc.robot.constants.FieldConstants.adjustedShopLayout;
import static frc.robot.constants.FieldConstants.compLayout;
import static frc.robot.vision.apriltag.VisionConstants.CAMERA_LOG_PATH;
import static frc.robot.vision.apriltag.VisionConstants.EXCLUDED_TAG_IDS;
import static frc.robot.vision.apriltag.VisionConstants.FIELD_BORDER_MARGIN;
import static frc.robot.vision.apriltag.VisionConstants.MAX_AMBIGUITY;
import static frc.robot.vision.apriltag.VisionConstants.THETA_STDDEV_MODEL;
import static frc.robot.vision.apriltag.VisionConstants.XY_STDDEV_MODEL;
import static frc.robot.vision.apriltag.VisionConstants.maxChangeDistance;
import static frc.robot.vision.apriltag.VisionConstants.multiTagModifier;
import static frc.robot.vision.apriltag.VisionConstants.stabilityModifier;
import static frc.robot.vision.apriltag.VisionConstants.thetaModifier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.constants.FieldConstants;
import frc.robot.commons.GremlinLogger;

public class GremlinApriltagVision extends SubsystemBase {
  private GremlinPhotonCamera[] cameras;
  private GremlinLimelightCamera[] limelights;
  private List<TimestampedVisionUpdate> visionUpdates;

  private PhotonCameraSim[] simCameras;
  private VisionSystemSim visionSystemSim;
  private SimCameraProperties[] simCameraProperties;

  private static final AprilTagFieldLayout LAYOUT = FieldConstants.isShopField ? adjustedShopLayout : compLayout;
  private static final double fieldLength = LAYOUT.getFieldLength();
  private static final double fieldWidth = LAYOUT.getFieldWidth();

  // Will be the function in drivetrain that adds vision estimate to pose
  // estimation
  private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (visionUpdates) -> {
  };
  // Will be the function in driveTrain that supplies current pose estimate
  private Supplier<Pose2d> poseSupplier = () -> new Pose2d();

  private boolean shouldRejectAllUpdates;

  /** Creates a new GremlinApriltagVision. */
  public GremlinApriltagVision(
      GremlinPhotonCamera[] cameras,
      Supplier<Pose2d> poseSupplier,
      Consumer<List<TimestampedVisionUpdate>> visionConsumer) {

    this.cameras = cameras;
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
    this.limelights = new GremlinLimelightCamera[0];

    shouldRejectAllUpdates = false;

    if (Utils.isSimulation()) {
      configSim();
    }
  }

  /** Creates a new GremlinApriltagVision. */
  public GremlinApriltagVision(
      GremlinPhotonCamera[] cameras,
      Supplier<Pose2d> poseSupplier,
      GremlinLimelightCamera[] limelights,
      Consumer<List<TimestampedVisionUpdate>> visionConsumer) {

    this.cameras = cameras;
    this.poseSupplier = poseSupplier;
    this.visionConsumer = visionConsumer;
    this.limelights = limelights;

    shouldRejectAllUpdates = false;

    if (Utils.isSimulation()) {
      configSim();
    }
  }

  public void setRejectAllUpdates(boolean shouldRejectAllUpdates) {
    this.shouldRejectAllUpdates = shouldRejectAllUpdates;
    GremlinLogger.debugLog("Reject Global Updates", shouldRejectAllUpdates);
  }

  @Override
  public void periodic() {
    if (!shouldRejectAllUpdates) {
      processVisionUpdates();
      visionConsumer.accept(visionUpdates);
    } 

    logLimelights();
  }

  @SuppressWarnings("unused")
  public void processVisionUpdates() {
    // Reset VisionUpdates
    visionUpdates = new ArrayList<>();
    // Get current robot position
    Pose2d currentPose = poseSupplier.get();

    // loop through all cameras
    for (int i = 0; i < cameras.length; i++) {
      String logPath = CAMERA_LOG_PATH + cameras[i].getName();

      // Camera specific variables
      Transform3d camToRobotTransform = GeomUtil.pose3dToTransform3d(cameras[i].getCameraPose()).inverse();
      List<PhotonPipelineResult> unreadResults = cameras[i].getAllUnreadResults();
      GremlinLogger.debugLog(logPath + "/Unread results", unreadResults.size());

      for (int j = 0; j < unreadResults.size(); j++) {
        Pose3d cameraPose;
        Pose2d calculatedRobotPose;
        List<Pose3d> tagPose3ds = new ArrayList<>();
        PhotonPipelineResult unprocessedResult = unreadResults.get(j);
        double timestamp = unprocessedResult.getTimestampSeconds();
        double singleTagAdjustment = 1.0;

        GremlinLogger.debugLog(logPath + "/Hastargets", unprocessedResult.hasTargets());
        GremlinLogger.debugLog(logPath + "/Timestamp", timestamp);

        // Continue if the camera doesn't have any targets
        if (!unprocessedResult.hasTargets()) {
          continue;
        }

        // if it has a MultiTag result we prefer to use that
        boolean shouldUseMultiTag = unprocessedResult.getMultiTagResult().isPresent();

        if (shouldUseMultiTag) {
          // TODO: think about adding processing to compare best and alt
          cameraPose = GeomUtil.transform3dToPose3d(unprocessedResult.getMultiTagResult().get().estimatedPose.best);

          calculatedRobotPose = cameraPose.transformBy(camToRobotTransform).toPose2d();

          // Populate array of tag poses with tags used
          for (int id : unprocessedResult.getMultiTagResult().get().fiducialIDsUsed) {
            tagPose3ds.add(LAYOUT.getTagPose(id).get());
          }

          GremlinLogger.debugLog(logPath + "/Multitag", true);
        } else {
          PhotonTrackedTarget target = unprocessedResult.getBestTarget();

          // We dont like some tags
          if (EXCLUDED_TAG_IDS.contains(target.getFiducialId()))
            continue;

          Pose3d tagPose = LAYOUT.getTagPose(target.getFiducialId()).get();

          Pose3d bestCamPose = tagPose.transformBy(target.getBestCameraToTarget().inverse());
          Pose3d altCamPose = tagPose.transformBy(target.getAlternateCameraToTarget().inverse());
          Pose2d bestRobotPose = bestCamPose.transformBy(camToRobotTransform).toPose2d();
          Pose2d altRobotPose = altCamPose.transformBy(camToRobotTransform).toPose2d();

          double ambiguity = target.getPoseAmbiguity();
          boolean betterRotationDiff = (Math.abs(bestRobotPose.getRotation().minus(currentPose.getRotation())
              .getRadians()) < Math.abs(altRobotPose.getRotation().minus(currentPose.getRotation()).getRadians()));

          if (ambiguity < MAX_AMBIGUITY) {
            // Best pose is significantly better than alt
            cameraPose = bestCamPose;
            calculatedRobotPose = bestRobotPose;
          } else if (betterRotationDiff) {
            // the rotation based on best pose is closer to our current estimated rotation
            cameraPose = bestCamPose;
            calculatedRobotPose = bestRobotPose;
          } else {
            cameraPose = altCamPose;
            calculatedRobotPose = altRobotPose;
          }

          tagPose3ds.add(tagPose);
          singleTagAdjustment = SingleTagAdjusters.getAdjustmentForTag(target.getFiducialId());

          GremlinLogger.debugLog(logPath + "/Multitag", false);
        }

        if (cameraPose == null || calculatedRobotPose == null)
          continue;

        // Move on to next camera if robot pose is off the field
        if (calculatedRobotPose.getX() < -FIELD_BORDER_MARGIN
            || calculatedRobotPose.getX() > fieldLength + FIELD_BORDER_MARGIN
            || calculatedRobotPose.getY() < -FIELD_BORDER_MARGIN
            || calculatedRobotPose.getY() > fieldWidth + FIELD_BORDER_MARGIN) {
          continue;
        }

        // To promote stability throw out poses that are too far from the last pose
        if (calculatedRobotPose.getTranslation()
            .getDistance(poseSupplier.get().getTranslation()) > maxChangeDistance)
          continue;

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPose3ds) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }

        double avgDistance = totalDistance / tagPose3ds.size();
        double xyStdDev = 0.0;
        double thetaStdDev = 0.0;

        if (shouldUseMultiTag) {
          xyStdDev = XY_STDDEV_MODEL.predict(avgDistance) * stabilityModifier * multiTagModifier;
          thetaStdDev = THETA_STDDEV_MODEL.predict(avgDistance) * stabilityModifier * multiTagModifier;
        } else {
          xyStdDev = XY_STDDEV_MODEL.predict(avgDistance) * stabilityModifier;
          thetaStdDev = THETA_STDDEV_MODEL.predict(avgDistance) * stabilityModifier;
        }

        Vector<N3> stdDevs = VecBuilder.fill(
            xyStdDev,
            xyStdDev,
            thetaStdDev * thetaModifier);

        if (!shouldUseMultiTag) {
          stdDevs.times(singleTagAdjustment);
        }

        visionUpdates.add(
            new TimestampedVisionUpdate(
                calculatedRobotPose,
                timestamp,
                stdDevs));

        logPoses(i, cameraPose, calculatedRobotPose, tagPose3ds.toArray(Pose3d[]::new));
        GremlinLogger.debugLog(logPath + "/TagsUsed", tagPose3ds.size());
        GremlinLogger.debugLog(logPath + "/StdDevs", stdDevs);
      }
    }
  }

  private static final StructPublisher<Pose3d> FLcamPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "topLeft" + "/Camera Pose", Pose3d.struct).publish();
  private static final StructPublisher<Pose2d> FLcalculatedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "topLeft" + "/Calculated Pose", Pose2d.struct).publish();
  private static final StructArrayPublisher<Pose3d> FLtagPosesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic(CAMERA_LOG_PATH + "topLeft" + "/Tag Poses", Pose3d.struct).publish();
  private static final StructPublisher<Pose3d> FRcamPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "topRight" + "/Camera Pose", Pose3d.struct).publish();
  private static final StructPublisher<Pose2d> FRcalculatedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "topRight" + "/Calculated Pose", Pose2d.struct).publish();
  private static final StructArrayPublisher<Pose3d> FRtagPosesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic(CAMERA_LOG_PATH + "topRight" + "/Tag Poses", Pose3d.struct).publish();
  private static final StructPublisher<Pose3d> BLcamPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "backLeft" + "/Camera Pose", Pose3d.struct).publish();
  private static final StructPublisher<Pose2d> BLcalculatedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "backLeft" + "/Calculated Pose", Pose2d.struct).publish();
  private static final StructArrayPublisher<Pose3d> BLtagPosesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic(CAMERA_LOG_PATH + "backLeft" + "/Tag Poses", Pose3d.struct).publish();
  private static final StructPublisher<Pose3d> BRcamPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "backRight" + "/Camera Pose", Pose3d.struct).publish();
  private static final StructPublisher<Pose2d> BRcalculatedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "backRight" + "/Calculated Pose", Pose2d.struct).publish();
  private static final StructArrayPublisher<Pose3d> BRtagPosesPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic(CAMERA_LOG_PATH + "backRight" + "/Tag Poses", Pose3d.struct).publish();
  private static final StructPublisher<Pose2d> LLlleftCalculatedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "limelightLeft" + "/Calculated Pose", Pose2d.struct).publish();
  private static final StructPublisher<Pose2d> LLrightCalculatedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "limelightRight" + "/Calculated Pose", Pose2d.struct).publish();
  private static final StructPublisher<Pose2d> LLlleftMT1PosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "limelightLeft" + "/MT1 Pose", Pose2d.struct).publish();
  private static final StructPublisher<Pose2d> LLrightMT1PosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic(CAMERA_LOG_PATH + "limelightRight" + "/MT1 Pose", Pose2d.struct).publish();

  /**
   * Log the CamPose, Calculated Pose, and TagPose
   * 
   * @param camID
   */
  private void logPoses(int camID, Pose3d camPose, Pose2d calculatedPose, Pose3d[] tagPoses) {
    switch (camID) {
      case 0:
        FLcamPosePublisher.set(camPose);
        FLcalculatedPosePublisher.set(calculatedPose);
        FLtagPosesPublisher.set(tagPoses);
        GremlinLogger.log("Cameras/topleft", camPose);
        break;
      case 1:
        FRcamPosePublisher.set(camPose);
        FRcalculatedPosePublisher.set(calculatedPose);
        FRtagPosesPublisher.set(tagPoses);
        GremlinLogger.log("Cameras/topRight", camPose);
        break;
      case 2:
        BLcamPosePublisher.set(camPose);
        BLcalculatedPosePublisher.set(calculatedPose);
        BLtagPosesPublisher.set(tagPoses);
        GremlinLogger.log("Cameras/backLeft", camPose);
        break;
      case 3:
        BRcamPosePublisher.set(camPose);
        BRcalculatedPosePublisher.set(calculatedPose);
        BRtagPosesPublisher.set(tagPoses);
        GremlinLogger.log("Cameras/backRight", camPose);
        break;
    }
  }

  private void logLimelights() {
    if (limelights[0].getBotPoseEstimateMT2().isPresent() && limelights[0].getBotPoseEstimateMT2().get().pose != null) {
      GremlinLogger.log("Limelights/Right", limelights[0].getBotPoseEstimateMT2().get().pose);
      LLrightCalculatedPosePublisher.set(limelights[0].getBotPoseEstimateMT2().get().pose);
      //LLrightMT1PosePublisher.set(limelights[0].getBotPoseEstimate().get().pose);
    }

    if (limelights[1].getBotPoseEstimateMT2().isPresent() && limelights[1].getBotPoseEstimateMT2().get().pose != null) {
      GremlinLogger.log("Limelights/Left", limelights[0].getBotPoseEstimateMT2().get().pose);
      LLlleftCalculatedPosePublisher.set(limelights[1].getBotPoseEstimateMT2().get().pose);
      //LLlleftMT1PosePublisher.set(limelights[1].getBotPoseEstimate().get().pose);
    }
  }

  public void configSim() {
    visionSystemSim = new VisionSystemSim("ApriltagVision");
    visionSystemSim.addAprilTags(LAYOUT);

    simCameras = new PhotonCameraSim[cameras.length + limelights.length];
    simCameraProperties = new SimCameraProperties[cameras.length + limelights.length];

    for (int i = 0; i < cameras.length; i++) {
      simCameraProperties[i] = VisionConstants.getOV2311();
      simCameras[i] = new PhotonCameraSim(cameras[i].getPhotonCamera(), simCameraProperties[i]);
      simCameras[i].enableDrawWireframe(false);
      simCameras[i].enableRawStream(false);
      simCameras[i].enableProcessedStream(false);
      visionSystemSim.addCamera(simCameras[i], GeomUtil.pose3dToTransform3d(cameras[i].getCameraPose()));
    }

    for (int i = 0; i < limelights.length; i++) {
      int position = cameras.length + i;
      simCameraProperties[position] = VisionConstants.getLL3();
      simCameras[position] = new PhotonCameraSim(limelights[i].getPhotonCamera(), simCameraProperties[position]);
      simCameras[position].enableDrawWireframe(false);
      simCameras[position].enableRawStream(false);
      simCameras[position].enableProcessedStream(false);
      simCameras[position].setTargetSortMode(PhotonTargetSortMode.Largest);
      visionSystemSim.addCamera(simCameras[position], GeomUtil.pose3dToTransform3d(limelights[i].getCameraPose()));
    }
  }

  @Override
  public void simulationPeriodic() {
    visionSystemSim.update(poseSupplier.get());
    Field2d debugField = visionSystemSim.getDebugField();
    debugField.getObject("EstimatedRobot").setPose(poseSupplier.get());
    debugField.getRobotObject().setPose(poseSupplier.get());

    for (GremlinLimelightCamera ll : limelights)
      ll.processSimUpdates();

    processVisionUpdates();

    visionConsumer.accept(visionUpdates);
    GremlinLogger.debugLog("VISION/visionUpdatesSize", visionUpdates.size());
  }
}
