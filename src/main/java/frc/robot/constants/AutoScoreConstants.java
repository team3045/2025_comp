package frc.robot.constants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoScoreConstants {
    public static final double kElevatorHeights[] = { 0, 0, 0, 0 };
    public static final double kPivotAngles[] = { 0, 0, 0, 0 };
    public static final double kMaxVelError = 0.1;
    public static final double kMaxPathFindTranslationError = 0.2;
    public static final double kElevatorResetHeight = 0;
    public static final double kPivotResetAngle = 0;
    public static final double kBackUpDist = 0.1;

    public static final double translationkP = 8;
    public static final double translationkI = 0;
    public static final double translationkD = 0;
    public static final double rotationkP = 8;
    public static final double rotationkI = 0;
    public static final double rotationkD = 0;

    public static final double basicPIDDistance = 2;

    // Pole Number, Scoring Pose2d
    public static HashMap<Integer, Pose2d> kScorePoseMap = new HashMap<Integer, Pose2d>();
    public static HashMap<Integer, Double> kScoreHeightMap = new HashMap<Integer, Double>();
    public static HashMap<Integer, Double> kScoreAngleMap = new HashMap<Integer, Double>();
    public static HashMap<Integer, Double> kScoreAngleMapAuto = new HashMap<Integer, Double>();
    public static HashMap<Integer, Integer> kBlueApriltagMap = new HashMap<>();
    public static HashMap<Integer, Integer> kRedApriltagMap = new HashMap<>();
    public static HashMap<Integer, Pose2d> kScorePoseMapV2 = new HashMap<>();

    public static List<Pose2d> rightScorePoses = new ArrayList<Pose2d>();
    public static List<Pose2d> leftScorePoses = new ArrayList<Pose2d>();
    public static List<Pose2d> flippedRightScorePoses = new ArrayList<>();
    public static List<Pose2d> flippedLeftScorePoses = new ArrayList<>();

    public static int[] kReefAprilTagIds = {
            6, 7, 8, 9, 10, 11, // RED
            17, 18, 19, 20, 21, 22 // BLUE
    };

    static {
        kScorePoseMap.put(9, new Pose2d(5.31, 5.05, Rotation2d.fromDegrees(-120)));
        kScorePoseMap.put(10, new Pose2d(4.99, 5.25, Rotation2d.fromDegrees(-120)));
        kScorePoseMap.put(11, new Pose2d(4.01, 5.25, Rotation2d.fromDegrees(-60)));
        kScorePoseMap.put(12, new Pose2d(3.68, 5.06, Rotation2d.fromDegrees(-60)));
        kScorePoseMap.put(1, new Pose2d(3.18, 4.21, Rotation2d.kZero));
        kScorePoseMap.put(2, new Pose2d(3.17, 3.850, Rotation2d.kZero));
        kScorePoseMap.put(3, new Pose2d(3.66, 3, Rotation2d.fromDegrees(60)));
        kScorePoseMap.put(4, new Pose2d(3.95, 2.81, Rotation2d.fromDegrees(60)));
        kScorePoseMap.put(5, new Pose2d(4.93, 2.8, Rotation2d.fromDegrees(120)));
        kScorePoseMap.put(6, new Pose2d(5.287, 2.96, Rotation2d.fromDegrees(120)));
        kScorePoseMap.put(7, new Pose2d(5.811, 3.83, Rotation2d.k180deg));
        kScorePoseMap.put(8, new Pose2d(5.811, 4.20, Rotation2d.k180deg)); // TODO: flip all these based on alliance
                                                                            // color

        kScoreHeightMap.put(1, ElevatorPivotConstants.HeightPositions.L2.getHeight());
        kScoreHeightMap.put(2, ElevatorPivotConstants.HeightPositions.L3.getHeight());
        kScoreHeightMap.put(3, ElevatorPivotConstants.HeightPositions.L4_V2.getHeight());

        kScoreAngleMap.put(1, ElevatorPivotConstants.AnglePositions.L2.getAngle());
        kScoreAngleMap.put(2, ElevatorPivotConstants.AnglePositions.L3.getAngle());
        kScoreAngleMap.put(3, ElevatorPivotConstants.AnglePositions.L4_V2.getAngle());

        kScoreAngleMapAuto.put(3, ElevatorPivotConstants.AnglePositions.L4_AUTO.getAngle());

        kBlueApriltagMap.put(1, 20);
        kBlueApriltagMap.put(2, 20);
        kBlueApriltagMap.put(3, 19);
        kBlueApriltagMap.put(4, 19);
        kBlueApriltagMap.put(5, 18);
        kBlueApriltagMap.put(6, 18);
        kBlueApriltagMap.put(7, 17);
        kBlueApriltagMap.put(8, 17);
        kBlueApriltagMap.put(9, 22);
        kBlueApriltagMap.put(10, 22);
        kBlueApriltagMap.put(11, 21);
        kBlueApriltagMap.put(12, 21);

        kRedApriltagMap.put(1, 11);
        kRedApriltagMap.put(2, 11);
        kRedApriltagMap.put(3, 6);
        kRedApriltagMap.put(4, 6);
        kRedApriltagMap.put(5, 7);
        kRedApriltagMap.put(6, 7);
        kRedApriltagMap.put(7, 8);
        kRedApriltagMap.put(8, 8);
        kRedApriltagMap.put(9, 9);
        kRedApriltagMap.put(10, 9);
        kRedApriltagMap.put(11, 10);
        kRedApriltagMap.put(12, 10);

        kScorePoseMapV2.put(1, new Pose2d(3.08,4.19, Rotation2d.kZero));
        kScorePoseMapV2.put(2, new Pose2d(3.08,3.86, Rotation2d.kZero));
        kScorePoseMapV2.put(3, new Pose2d(3.67,2.89, Rotation2d.fromDegrees(60)));
        kScorePoseMapV2.put(4, new Pose2d(3.94,2.72, Rotation2d.fromDegrees(60)));
        kScorePoseMapV2.put(5, new Pose2d(5.06,2.72, Rotation2d.fromDegrees(120)));
        kScorePoseMapV2.put(6, new Pose2d(5.35,2.89, Rotation2d.fromDegrees(120)));
        kScorePoseMapV2.put(7, new Pose2d(5.92,3.87, Rotation2d.k180deg));
        kScorePoseMapV2.put(8, new Pose2d(5.92,4.19, Rotation2d.k180deg));
        kScorePoseMapV2.put(9, new Pose2d(5.33,5.18, Rotation2d.fromDegrees(-120)));
        kScorePoseMapV2.put(10, new Pose2d(5.05,5.35, Rotation2d.fromDegrees(-120)));
        kScorePoseMapV2.put(11, new Pose2d(3.94,5.35, Rotation2d.fromDegrees(-60)));
        kScorePoseMapV2.put(12, new Pose2d(3.65,5.16, Rotation2d.fromDegrees(-60)));

        List<Integer> sortedKeys = new ArrayList<>(kScorePoseMap.keySet());
            Collections.sort(sortedKeys);
        
        for (Integer key : sortedKeys) {
            if (key % 2 == 0) {
                rightScorePoses.add(kScorePoseMapV2.get(key));
            } else {
                leftScorePoses.add(kScorePoseMapV2.get(key)); 
            }
        }

        flippedRightScorePoses = rightScorePoses.stream()
                        .map((pose) -> FlippingUtil.flipFieldPose(pose)).toList();
        flippedLeftScorePoses = leftScorePoses.stream()
                        .map((pose) -> FlippingUtil.flipFieldPose(pose)).toList();
    }
}
