// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.apriltag;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.commons.PolynomialRegression;

/** Add your docs here. */
public class VisionConstants {
        // private constructor so cant be instantiated
        private VisionConstants() {
        }

        public static final String CAMERA_LOG_PATH = "VISION/CAMERAS/";

        public static final Set<Integer> EXCLUDED_TAG_IDS = new HashSet<>(Arrays.asList());

        public static final double MAX_AMBIGUITY = 0.3;
        public static final double FIELD_BORDER_MARGIN = 0.5;
        public static final int OV2311_RES_HORIZONTAL = 1600;
        public static final int OV2311_RES_VERTICAL = 1200;
        public static final double OV2311_FOV_DIAG = 90;
        public static final int LL3_RES_HORIZONTAL = 1280;
        public static final int LL3_RES_VERTICAL = 960;
        public static final double LL3_FOV_DIAG = 90;

        public static final PolynomialRegression XY_STDDEV_MODEL = new PolynomialRegression(
                        new double[] {
                                        0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                                        3.223358, 4.093358, 4.726358
                        },
                        new double[] { 0.005, 0.0135, 0.016, 0.038, 0.0515, 0.0925, 0.12, 0.14, 0.17, 0.27, 0.38 },
                        2);
        public static final PolynomialRegression THETA_STDDEV_MODEL = new PolynomialRegression(
                        new double[] {
                                        0.752358, 1.016358, 1.296358, 1.574358, 1.913358, 2.184358, 2.493358, 2.758358,
                                        3.223358, 4.093358, 4.726358
                        },
                        new double[] { 0.008, 0.027, 0.015, 0.044, 0.04, 0.078, 0.049, 0.027, 0.059, 0.029, 0.068 },
                        1);

        public static final double thetaModifier = 100;
        public static final double multiTagModifier = 0.6;
        public static final double stabilityModifier = 15;
        public static final double maxChangeDistance = 3; // m

        public static SimCameraProperties getOV2311() {
                SimCameraProperties properties = new SimCameraProperties();
                properties.setCalibration(OV2311_RES_HORIZONTAL, OV2311_RES_VERTICAL,
                                Rotation2d.fromDegrees(OV2311_FOV_DIAG));
                properties.setCalibError(0.25, 0.08);
                properties.setFPS(25);
                properties.setAvgLatencyMs(35);
                properties.setLatencyStdDevMs(5);

                return properties;
        }

        public static SimCameraProperties getLL3() {
                SimCameraProperties properties = new SimCameraProperties();
                properties.setCalibration(LL3_RES_HORIZONTAL, LL3_RES_VERTICAL, Rotation2d.fromDegrees(LL3_FOV_DIAG));
                properties.setCalibError(0.25, 0.08);
                properties.setFPS(25);
                properties.setAvgLatencyMs(35);
                properties.setLatencyStdDevMs(5);

                return properties;

        }

        public static final Pose3d[] cameraPoses = {
                        new Pose3d( // Front Left
                                        new Translation3d(
                                                        Units.inchesToMeters(10.886 + 0.75),
                                                        Units.inchesToMeters(9.362 + 0.75),
                                                        Units.inchesToMeters(8.398)),
                                        new Rotation3d(0, Units.degreesToRadians(-28.125), Units.degreesToRadians(30))),
                        new Pose3d( // Front Right
                                        new Translation3d(
                                                        Units.inchesToMeters(10.886 + 0.75),
                                                        -Units.inchesToMeters(9.362 + 0.75),
                                                        Units.inchesToMeters(8.398)),
                                        new Rotation3d(0, Units.degreesToRadians(-28.125),
                                                        Units.degreesToRadians(-30))),
                        new Pose3d( // Back Left
                                        new Translation3d(
                                                        -Units.inchesToMeters(10.886 + 0.75),
                                                        Units.inchesToMeters(9.362 + 0.75),
                                                        Units.inchesToMeters(8.398)),
                                        new Rotation3d(0, Units.degreesToRadians(-28.125),
                                                        Units.degreesToRadians(150))),
                        new Pose3d( // Back Right
                                        new Translation3d(
                                                        -Units.inchesToMeters(10.886 + 0.75),
                                                        -Units.inchesToMeters(9.362 + 0.75),
                                                        Units.inchesToMeters(8.398)),
                                        new Rotation3d(0, Units.degreesToRadians(-28.125),
                                                        Units.degreesToRadians(-150))),
        };

        public static final Pose3d[] limelightPoses = {
                        new Pose3d( // right
                                        new Translation3d(
                                                        Units.inchesToMeters(8.055),
                                                        Units.inchesToMeters(6.35),
                                                        Units.inchesToMeters(6.9205)),
                                        new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(7.5))),
                        new Pose3d(
                                        new Translation3d(
                                                        Units.inchesToMeters(8.055),
                                                        Units.inchesToMeters(-6.35),
                                                        Units.inchesToMeters(6.9205)),
                                        new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(-7.5))) // left
        };

        public static final GremlinPhotonCamera[] cameras = { // Retain this Order
                        // new GremlinPhotonCamera(NetworkTableInstance.getDefault(), "frontLeft", cameraPoses[0]),
                        // new GremlinPhotonCamera(NetworkTableInstance.getDefault(), "frontRight", cameraPoses[1]),
                        // new GremlinPhotonCamera(NetworkTableInstance.getDefault(), "backLeft", cameraPoses[2]),
                        // new GremlinPhotonCamera(NetworkTableInstance.getDefault(), "backRight", cameraPoses[3])
        };

        public static final GremlinLimelightCamera[] limelights = { // retain this order
                        new GremlinLimelightCamera("limelight-right", limelightPoses[0]),
                        new GremlinLimelightCamera("limelight-left", limelightPoses[1])
        };
}
