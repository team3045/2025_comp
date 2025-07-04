package frc.robot.commands;

import static frc.robot.constants.DriveConstants.drive;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.GremlinAutoBuilder;
import frc.robot.commons.GremlinLogger;
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
  private Pose2d targetPose;

  private BooleanSupplier overrideWithFeedback;
  private boolean startedPId;

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
    startedPId = false;
    updatePathfindCommand();
    if (currentPathfindCommand != null) {
      currentPathfindCommand.initialize();
    }

    xController.setGoal(targetPoseSupplier.get().getX());
    yController.setGoal(targetPoseSupplier.get().getY());

  }

  @Override
  public void execute() {
    if (overrideWithFeedback.getAsBoolean()) {
      feedbackPosePublisher.set(feedbackPoseSupplier.get());
      if (!feedbackPoseSupplier.get().equals(Pose2d.kZero)) {
        drivetrain.addVisionMeasurement(
            feedbackPoseSupplier.get(),
            Utils.fpgaToCurrentTime(timestampSupplier.getAsDouble()),
            VecBuilder.fill(0.001, 0.001, 0.001));
      }
    } else {
      PPHolonomicDriveController.clearFeedbackOverrides();
    }

    if (currentPathfindCommand != null) {
      currentPathfindCommand.execute();
    }

    if(drivetrain.getState().Pose.getTranslation().getDistance(targetPose.getTranslation()) < 0.1 && !startedPId){
      updatePathfindCommand();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (currentPathfindCommand != null) {
      currentPathfindCommand.end(interrupted);
    }

    currentPathfindCommand = null;
    drivetrain.setControl(DriveConstants.brake);
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("At Pose",
        GeomUtil.isNearPose(targetPose, drivetrain.getState().Pose, DriveConstants.preciseTranslationTolerance));

    return GeomUtil.isNearPose(targetPose, drivetrain.getState().Pose, DriveConstants.preciseTranslationTolerance) 
      && drivetrain.getState().Speeds.vxMetersPerSecond < 0.05
      && drivetrain.getState().Speeds.vyMetersPerSecond < 0.05
      && drivetrain.getState().Speeds.omegaRadiansPerSecond < Units.degreesToRadians(1);
  }

  private void updatePathfindCommand() {
    // Get the dynamic pose and velocity
    targetPose = AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(targetPoseSupplier.get())
        : targetPoseSupplier.get();
    double desiredEndVelocity = desiredEndVelocitySupplier.getAsDouble();

    GremlinLogger.debugLog("DriveState/Target", targetPose);

    Robot.pathfinder.setGoalPosition(targetPose.getTranslation());
    Robot.pathfinder.setStartPosition(drivetrain.getState().Pose.getTranslation());

    if (drivetrain.getState().Pose.getTranslation()
        .getDistance(targetPose.getTranslation()) < AutoScoreConstants.basicPIDDistance) {
      currentPathfindCommand = drivetrain.preciseTargetPose(targetPoseSupplier);
      currentPathfindCommand.initialize();
      startedPId = true;
    } else {
      // Create a new pathfinding command with the updated values
      currentPathfindCommand = GremlinAutoBuilder.pathfindToPose(targetPose,
          constraints,
          desiredEndVelocity);
    }
  }
}
