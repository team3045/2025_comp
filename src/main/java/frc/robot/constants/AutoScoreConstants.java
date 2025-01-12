package frc.robot.constants;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoScoreConstants {
    public static final double kElevatorHeights[] = {0, 0, 0, 0};
    public static final double kPivotAngles[] = {0, 0, 0, 0};
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

    //Pole Number, Scoring Pose2d
    public static HashMap<Integer, Pose2d> kScorePoseMap = new HashMap<Integer, Pose2d>();
    public static HashMap<Integer, Double> kScoreHeightMap = new HashMap<Integer, Double>();
    public static HashMap<Integer, Double> kScoreAngleMap = new HashMap<Integer, Double>();

    static{
        kScorePoseMap.put(1, new Pose2d(2,7,new Rotation2d()));
        kScorePoseMap.put(2, new Pose2d(2,7,new Rotation2d()));
        kScorePoseMap.put(3, new Pose2d(2,7,new Rotation2d()));
        kScorePoseMap.put(4, new Pose2d(2,7,new Rotation2d()));
        kScorePoseMap.put(5, new Pose2d(2,7,new Rotation2d()));
        kScorePoseMap.put(6, new Pose2d(2,7,new Rotation2d()));
        kScorePoseMap.put(7, new Pose2d(2,7,new Rotation2d()));
        kScorePoseMap.put(8, new Pose2d(2,7,new Rotation2d()));
        kScorePoseMap.put(9, new Pose2d(2,7,new Rotation2d()));
        kScorePoseMap.put(10, new Pose2d(2,7,new Rotation2d()));
        kScorePoseMap.put(11, new Pose2d(2,7,new Rotation2d()));
        kScorePoseMap.put(12, new Pose2d(2,7,new Rotation2d()));

        kScoreHeightMap.put(1, ElevatorPivotConstants.minimumHeight);
        kScoreHeightMap.put(2, ElevatorPivotConstants.maxHeight);
        kScoreHeightMap.put(3, ElevatorPivotConstants.maxHeight);

        kScoreAngleMap.put(1, 0.0);
        kScoreAngleMap.put(2, 45.0);
        kScoreAngleMap.put(3, 90.0);
    }
}
