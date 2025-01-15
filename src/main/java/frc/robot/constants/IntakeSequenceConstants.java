package frc.robot.constants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

    public static final BoundingBox leftSubstation = new BoundingBox(0, 0, 0, 0);
    public static final BoundingBox rightSubstation = new BoundingBox(0, 0, 0, 0);

    public static final Pose2d leftSubstationPose =
            new Pose2d(
                new Translation2d(0, 0),  
                new Rotation2d(Math.toRadians(0)) 
            );

    public static final Pose2d rightSubstationPose = 
            new Pose2d(
                new Translation2d(0, 0),  
                new Rotation2d(Math.toRadians(0)) 
            );


    
     
}
