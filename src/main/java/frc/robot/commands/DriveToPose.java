package frc.robot.commands;

import static frc.robot.constants.DriveConstants.MAX_ANGULAR_ACCEL_AUTOSCORE;
import static frc.robot.constants.DriveConstants.MAX_ANGULAR_VELOCITY_AUTOSCORE;

import java.lang.Thread.State;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command to drive to a pose.
 */
public class DriveToPose extends Command {

  /** Default constraints are 90% of max speed, accelerate to full speed in 1/3 second */
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      DriveConstants.MAX_VELO_AUTOSCORE,
      DriveConstants.MAX_ACCEL_AUTOSCORE);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_VELOCITY_AUTOSCORE,
      MAX_ANGULAR_ACCEL_AUTOSCORE);

  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;
  private final HolonomicDriveController holonomicController;

  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final Supplier<Pose2d> goalPoseSupplier;

  private Pose2d goalPose;

  public static final StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("DriveState/targetPose", Pose2d.struct).publish();

  public DriveToPose(
        CommandSwerveDrivetrain drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
        Supplier<Pose2d> goalPoseSup) {
    this(drivetrainSubsystem, poseProvider, goalPoseSup, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS);
  }

  public DriveToPose(
        CommandSwerveDrivetrain drivetrainSubsystem,
        Supplier<Pose2d> poseProvider,
        Supplier<Pose2d> goalPoseSup,
        TrapezoidProfile.Constraints xyConstraints,
        TrapezoidProfile.Constraints omegaConstraints) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.goalPoseSupplier = goalPoseSup;

    xController = new PIDController(DriveConstants.preciseTranslationkP, DriveConstants.preciseRotationkI, DriveConstants.preciseTranslationkD);
    yController = new PIDController(DriveConstants.preciseTranslationkP, DriveConstants.preciseRotationkI, DriveConstants.preciseTranslationkD);
    xController.setTolerance(DriveConstants.preciseTranslationTolerance);
    yController.setTolerance(DriveConstants.preciseTranslationTolerance);
    thetaController = new ProfiledPIDController(DriveConstants.preciseRotationkP, DriveConstants.preciseRotationkI, DriveConstants.preciseRotationkD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(DriveConstants.preciseRotationTolerance));
    holonomicController = new HolonomicDriveController(xController, yController, thetaController);

    addRequirements(drivetrainSubsystem);
  }


  @Override
  public void initialize() {
    goalPose = goalPoseSupplier.get();
    if (AutoBuilder.shouldFlip()) {
      goalPose = FlippingUtil.flipFieldPose(goalPose);
      SmartDashboard.putString("TargetPose", goalPose.toString());
    }
    
    targetPosePublisher.set(goalPose);
  }

  public boolean atGoal() {
    return holonomicController.atReference();
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = holonomicController.calculate(poseProvider.get(), goalPoseSupplier.get(), DriveConstants.MAX_VELOCITY, goalPoseSupplier.get().getRotation());
    drivetrainSubsystem.setControl(DriveConstants.APPLY_FIELD_SPEEDS
        .withSpeeds(speeds));
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setControl(DriveConstants.brake);
  }

}