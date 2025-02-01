package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.GremlinAutoBuilder;
import frc.robot.commons.GremlinPathfindingCommand;
import frc.robot.constants.AutoScoreConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DynamicPathfindWithFeedback extends Command {
    private final Supplier<Pose2d> targetPoseSupplier;
    private final DoubleSupplier desiredEndVelocitySupplier;
    private final PathConstraints constraints;
    private CommandSwerveDrivetrain drivetrain;


    private static final ProfiledPIDController xController = new ProfiledPIDController(6, 0, 0, 
      new Constraints(DriveConstants.MAX_VELOCITY_AUTO, DriveConstants.MAX_ACCEL_AUTO), 0.02);
      private static final ProfiledPIDController yController = new ProfiledPIDController(6, 0, 0, 
      new Constraints(DriveConstants.MAX_VELOCITY_AUTO, DriveConstants.MAX_ACCEL_AUTO), 0.02);

    public static final StructPublisher<Pose2d> feedbackPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("VISION/LimelightPose", Pose2d.struct).publish();

    private Command currentPathfindCommand;

    private Supplier<Pose2d> feedbackPoseSupplier;
    private DoubleSupplier timestampSupplier;

    private BooleanSupplier overrideWithFeedback;

    public DynamicPathfindWithFeedback(
        Supplier<Pose2d> targetPoseSupplier,
        DoubleSupplier desiredEndVelocitySupplier,
        PathConstraints constraints,
        CommandSwerveDrivetrain drivetrain,
        Supplier<Pose2d> feedbackPoseSupplier,
        BooleanSupplier overrideWithFeedback,
        DoubleSupplier timestampSupplier) {

        this.targetPoseSupplier = targetPoseSupplier;
        this.desiredEndVelocitySupplier = desiredEndVelocitySupplier;
        this.constraints = constraints;
        this.overrideWithFeedback = overrideWithFeedback;
        this.feedbackPoseSupplier = feedbackPoseSupplier;
        this.drivetrain = drivetrain;
        this.timestampSupplier = timestampSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        updatePathfindCommand();
        if (currentPathfindCommand != null) {
            currentPathfindCommand.initialize();
        } 

        xController.setGoal(targetPoseSupplier.get().getX());
        yController.setGoal(targetPoseSupplier.get().getY());

        //TODO: add a check to make sure we're within field bounds, 
        //as well as the target pose is within field bounds other wise pathPlanner will fail I think
    }

    @Override
    public void execute() {
      if(overrideWithFeedback.getAsBoolean()){
        feedbackPosePublisher.set(feedbackPoseSupplier.get());
        drivetrain.addVisionMeasurement(
          feedbackPoseSupplier.get(), 
          Utils.fpgaToCurrentTime(timestampSupplier.getAsDouble()),
          VecBuilder.fill(0.001,0.001,0.001));
      } else {
        PPHolonomicDriveController.clearFeedbackOverrides();
      }

      if (currentPathfindCommand != null) {
            currentPathfindCommand.execute();
      }
    }

    @Override
    public void end(boolean interrupted) {
        if (currentPathfindCommand != null) {
            currentPathfindCommand.end(interrupted);
        }

        currentPathfindCommand = null;
    }

    @Override
    public boolean isFinished() {
      Pose2d targetPose = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? 
        targetPoseSupplier.get() : FlippingUtil.flipFieldPose(targetPoseSupplier.get());
      return GeomUtil.isNearPose(targetPose, drivetrain.getState().Pose, DriveConstants.preciseTranslationTolerance);
    }

    private void updatePathfindCommand() {
        // Get the dynamic pose and velocity
        Pose2d targetPose = AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(targetPoseSupplier.get()) 
          : targetPoseSupplier.get();
        double desiredEndVelocity = desiredEndVelocitySupplier.getAsDouble();

        Robot.pathfinder.setGoalPosition(targetPose.getTranslation());
        Robot.pathfinder.setStartPosition(drivetrain.getState().Pose.getTranslation());

        if(drivetrain.getState().Pose.getTranslation().getDistance(targetPose.getTranslation()) < AutoScoreConstants.basicPIDDistance){
           currentPathfindCommand = drivetrain.preciseTargetPose(targetPoseSupplier);
        } else {
          // Create a new pathfinding command with the updated values
          currentPathfindCommand = GremlinAutoBuilder.pathfindToPose(targetPose, 
            constraints, 
            desiredEndVelocity);
        }
    }

    private double calculateXFeedback(){
      Pose2d targetPose = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? 
        targetPoseSupplier.get() : FlippingUtil.flipFieldPose(targetPoseSupplier.get());

      SmartDashboard.putNumber("autoScore/setpointX", targetPose.getX());
      SmartDashboard.putNumber("autoScore/measuredX", feedbackPoseSupplier.get().getX());
      return xController.calculate(feedbackPoseSupplier.get().getX(),targetPose.getX());
    }

    private double calculateYFeedback(){
      Pose2d targetPose = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? 
        targetPoseSupplier.get() : FlippingUtil.flipFieldPose(targetPoseSupplier.get());
      
      SmartDashboard.putNumber("autoScore/setpointY", targetPose.getY());
      SmartDashboard.putNumber("autoScore/measuredY", feedbackPoseSupplier.get().getY());
      
      return yController.calculate(feedbackPoseSupplier.get().getY(), feedbackPoseSupplier.get().getY());
    }
}
