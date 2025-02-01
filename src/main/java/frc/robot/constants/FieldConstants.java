package frc.robot.constants;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

public class FieldConstants {
        public static final boolean isShopField = true;
        public static final AprilTagFieldLayout compLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        public static final double shopFieldLength = 7.89225625;
        public static final double shopFieldWidth = 4.68122;

        public static final double compFieldLength = compLayout.getFieldLength(); //TODO: change these bc field size changed from 2024
        public static final double compFieldWidth = compLayout.getFieldWidth();


        // AprilTag constants
        public static final double aprilTagWidth = 165.1 / 1000;
        public static final double wallOffset = 0.00635;
        public static final double edgeToTag = 0.0508 + (aprilTagWidth / 2) ; //2 inches to edge and half of width to center
        public static final double temp = 1.42160625 + edgeToTag;

        public static final Translation2d blueReefCenter = new Translation2d(4.5,4);
        public static final Translation2d redReefCenter = FlippingUtil.flipFieldPosition(blueReefCenter);
        public static final double reefDistanceTolerance = 2+1.25;

        public static final List<AprilTag> shopTags = List.of(
                new AprilTag(1, new Pose3d(
                        2.943225 + edgeToTag, 
                        wallOffset, 
                        0.5842 + edgeToTag, 
                        new Rotation3d(0,0,Math.PI/2))),
                new AprilTag(2, new Pose3d(
                        wallOffset,
                        3.7671375 + edgeToTag,
                        2.35029375 + edgeToTag,
                        new Rotation3d(0,0,0))),
                new AprilTag(6, new Pose3d(
                        wallOffset,
                        1.22475625 + edgeToTag,
                        1.58511875  + edgeToTag,
                        new Rotation3d(0,0,0))),
                new AprilTag(7, new Pose3d(
                        4.71090625 + edgeToTag,
                        wallOffset,
                        0.650875 + edgeToTag,
                        new Rotation3d(0,0,Math.PI/2))),
                new AprilTag(8, new Pose3d(
                        7.88114375 ,
                        2.57730625 + edgeToTag,
                        1.42160625 + edgeToTag,
                        new Rotation3d(0,0,Math.PI))),
                new AprilTag(10, new Pose3d(
                        3.5 ,
                        4.0,
                        0.22225,
                        new Rotation3d(0,0,-Math.PI/2))));
        //Stuff to transform shop field apriltags relative to apriltag 10 on our mock reef 

        private static final Pose3d fieldTag10Pose = FieldConstants.compLayout.getTagPose(10).orElse(new Pose3d());
        private static final Pose3d shopTag10Pose = shopTags.get(5).pose;

        private static final Transform3d transformToRealField = new Transform3d(shopTag10Pose, fieldTag10Pose);

        public static final List<AprilTag> adjustedShopTags = new ArrayList<>();

        static {  
                StructArrayPublisher<Pose3d> pub = NetworkTableInstance.getDefault().getStructArrayTopic("ShopTags", Pose3d.struct).publish();
                for (AprilTag tag : shopTags) {
                        Pose3d tagPose = tag.pose; 
                        Pose3d transformedPose = tagPose.rotateBy(transformToRealField.getRotation());
                        Translation3d transform = fieldTag10Pose.getTranslation().minus(shopTag10Pose.rotateBy(transformToRealField.getRotation()).getTranslation());
                        transformedPose = new Pose3d(transformedPose.getTranslation().plus(transform), transformedPose.getRotation());
                        adjustedShopTags.add(new AprilTag(tag.ID, transformedPose));
                }

                pub.set(adjustedShopTags.stream().map((apriltag) -> apriltag.pose).toArray(Pose3d[]::new));
        }

        public static final AprilTagFieldLayout shopLayout = new AprilTagFieldLayout(
                shopTags, 
                shopFieldLength, 
                shopFieldWidth);
        
        public static final AprilTagFieldLayout adjustedShopLayout = new AprilTagFieldLayout(
                adjustedShopTags,
                compFieldLength,
                compFieldWidth);
}
