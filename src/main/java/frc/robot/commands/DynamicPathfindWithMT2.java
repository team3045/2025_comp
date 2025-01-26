package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.apriltag.GremlinLimelightCamera;

public class DynamicPathfindWithMT2 extends Command {
    private final Supplier<Pose2d> targetPoseSupplier;
    private final DoubleSupplier desiredEndVelocitySupplier;
    private final PathConstraints constraints;

    private static final PIDController overideFeedbackController = DriveConstants.preciseTranslationController;

    private Command currentPathfindCommand;
    private CommandSwerveDrivetrain drivetrain;

    private Supplier<Pose2d> feedbackPoseSupplier;
    private BooleanSupplier overrideWithFeedback;
    private GremlinLimelightCamera camera;

    public DynamicPathfindWithMT2(
        Supplier<Pose2d> targetPoseSupplier,
        DoubleSupplier desiredEndVelocitySupplier,
        PathConstraints constraints,
        CommandSwerveDrivetrain drivetrain,
        GremlinLimelightCamera camera) {

        this.targetPoseSupplier = targetPoseSupplier;
        this.desiredEndVelocitySupplier = desiredEndVelocitySupplier;
        this.constraints = constraints;
        this.overrideWithFeedback = camera::seesObject;
        this.feedbackPoseSupplier = () -> camera.getBotPoseEstimate().pose;
        this.drivetrain = drivetrain;
        this.camera = camera;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        updatePathfindCommand();
        if (currentPathfindCommand != null) {
            currentPathfindCommand.initialize();
        } 

        //TODO: add a check to make sure we're within field bounds, 
        //as well as the target pose is within field bounds other wise pathPlanner will fail I think
    }

    @Override
    public void execute() {
      camera.setRobotHeading(drivetrain.getState().Pose.getRotation().getDegrees());

      if(overrideWithFeedback.getAsBoolean()){
        PPHolonomicDriveController.overrideXYFeedback(this::calculateXFeedback, this::calculateYFeedback);
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

        PPHolonomicDriveController.clearFeedbackOverrides();
    }

    @Override
    public boolean isFinished() {
        return currentPathfindCommand == null || currentPathfindCommand.isFinished();
    }

    private void updatePathfindCommand() {
        // Get the dynamic pose and velocity
        Pose2d targetPose = targetPoseSupplier.get();
        double desiredEndVelocity = desiredEndVelocitySupplier.getAsDouble();

        // Create a new pathfinding command with the updated values
        currentPathfindCommand = AutoBuilder.pathfindToPoseFlipped(targetPose, constraints, desiredEndVelocity);
    }

    private double calculateXFeedback(){
      Pose2d targetPose = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? 
        targetPoseSupplier.get() : FlippingUtil.flipFieldPose(targetPoseSupplier.get());
        System.out.println("X: " + targetPose.getX());
      return overideFeedbackController.calculate(feedbackPoseSupplier.get().getX(),targetPose.getX());
    }

    private double calculateYFeedback(){
      Pose2d targetPose = DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue) ? 
        targetPoseSupplier.get() : FlippingUtil.flipFieldPose(targetPoseSupplier.get());
      System.out.println(targetPose.getX());
      return overideFeedbackController.calculate(feedbackPoseSupplier.get().getY(),targetPose.getY());
    }
}
