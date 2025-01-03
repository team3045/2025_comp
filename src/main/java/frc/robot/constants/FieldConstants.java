package frc.robot.constants;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class FieldConstants {
        public static final boolean isShopField = true;

        public static final double shopFieldLength = 7.89225625;
        public static final double shopFieldWidth = 4.68122;

        public static final double compFieldLength = 16.451;
        public static final double compFieldWidth = 8.211;


        // AprilTag constants
        public static final double aprilTagWidth = 165.1 / 1000;
        public static final double wallOffset = 0.00635;
        public static final double edgeToTag = 0.0508 + (aprilTagWidth / 2) ; //2 inches to edge and half of width to center
        public static final double temp = 1.42160625 + edgeToTag;
        
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
                        new Rotation3d(0,0,Math.PI))));

        public static final AprilTagFieldLayout shopLayout = new AprilTagFieldLayout(
                shopTags, 
                shopFieldLength, 
                shopFieldWidth);
        public static final AprilTagFieldLayout compLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
}
