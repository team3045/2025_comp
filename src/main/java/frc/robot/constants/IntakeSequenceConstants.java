package frc.robot.constants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class IntakeSequenceConstants {
    public static final DoubleSupplier intakeReadyHeight = () -> 1.5; 
    public static final DoubleSupplier intakeReadyAngle = () -> 45.0;

    public static final DoubleSupplier intakingHeight = () -> 1.5; 

    public static final DoubleSupplier stowHeight = () -> 0;
    public static final DoubleSupplier stowAngle = () -> 0;

    
    public static final DoubleSupplier desiredEndVelocity = () -> 0;

    public static final double timeOutTime = 5; 

    public static final Supplier<Pose2d> leftSubstationPose = () -> 
            new Pose2d(
                new Translation2d(0, 0),  
                new Rotation2d(Math.toRadians(0)) 
            );

    public static final Supplier<Pose2d> rightSubstationPose = () -> 
            new Pose2d(
                new Translation2d(0, 0),  
                new Rotation2d(Math.toRadians(0)) 
            );


    
     
}
