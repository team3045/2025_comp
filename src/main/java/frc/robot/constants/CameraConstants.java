package frc.robot.constants;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConstants {
    public static final Translation3d[] cameraOffsets = {new Translation3d(0.0, 0.0, 0.0), new Translation3d(0.0, 0.0, 0.0), new Translation3d(0.0, 0.0, 0.0)};
    public static final Rotation3d[] cameraRotations = {new Rotation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)};
    public static final Transform3d[] cameraTransforms = {new Transform3d(cameraOffsets[0], cameraRotations[0]), new Transform3d(cameraOffsets[1], cameraRotations[1]), new Transform3d(cameraOffsets[2], cameraRotations[2])};
    public static final String[] cameraNames = {new String("front_left"), new String("back_left"), new String("back_right")};
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    public static final PoseStrategy strategy = PoseStrategy.LOWEST_AMBIGUITY;
}