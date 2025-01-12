package frc.robot.constants;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;

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
        kScorePoseMap.put(1, Pose2d.kZero);
        kScorePoseMap.put(2, Pose2d.kZero);
        kScorePoseMap.put(3, Pose2d.kZero);
        kScorePoseMap.put(4, Pose2d.kZero);
        kScorePoseMap.put(5, Pose2d.kZero);
        kScorePoseMap.put(6, Pose2d.kZero);
        kScorePoseMap.put(7, Pose2d.kZero);
        kScorePoseMap.put(8, Pose2d.kZero);
        kScorePoseMap.put(9, Pose2d.kZero);
        kScorePoseMap.put(10, Pose2d.kZero);
        kScorePoseMap.put(11, Pose2d.kZero);
        kScorePoseMap.put(12, Pose2d.kZero);

        kScoreHeightMap.put(1, ElevatorPivotConstants.minimumHeight);
        kScoreHeightMap.put(2, ElevatorPivotConstants.minimumHeight);
        kScoreHeightMap.put(3, ElevatorPivotConstants.minimumHeight);

        kScoreAngleMap.put(1, 0.0);
        kScoreAngleMap.put(2, 0.0);
        kScoreAngleMap.put(3, 0.0);
    }
}
