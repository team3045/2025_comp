package frc.robot.constants;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commons.BoundingBox;

public class IntakeSequenceConstants {
    public static final double intakeReadyHeight =  1.5; 
    public static final double intakeReadyAngle =  45.0;

    public static final double stowHeight = 0;
    public static final double stowAngle =  0;

    public static final double intakingHeight =  1.5; 
    
    public static final double desiredEndVelocity = 0;

    public static final double timeOutTime = 5; 

    public static final BoundingBox topSubstation = new BoundingBox(0.058, 3.620, 5.302,8.023);
    public static final BoundingBox bottomSubstation = new BoundingBox(0.036, 3.057, 0, 2.353); //TODO: flip all these based on alliance

    public static final Pose2d topSubstationPose =
            new Pose2d(
                new Translation2d(1.343, 7.040),  
                Rotation2d.fromDegrees(128.830) 
            );

    public static final Pose2d bottomSubstationPose = 
            new Pose2d(
                new Translation2d(1.163, 1.142),  
                Rotation2d.fromDegrees(-126.254)
            );


    
     
}
