package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.GremlinLogger;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

public class DriveToPoseV2 extends Command {
  private static final double drivekP = 0.8;
  private static final double drivekD = 0;
  private static final double thetakP = 4;
  private static final double thetakD = 0;
  private static final double driveMaxVelocity =
      DriveConstants.MAX_VELO_AUTOSCORE;
  private static final double driveMaxAcceleration =
      DriveConstants.MAX_ACCEL_AUTOSCORE;
  private static final double thetaMaxVelocity =
      DriveConstants.MAX_ANGULAR_VELOCITY_AUTOSCORE;
  private static final double thetaMaxAcceleration =
      DriveConstants.MAX_ANGULAR_ACCEL_AUTOSCORE;
  private static final double driveTolerance =
      DriveConstants.preciseTranslationTolerance;
  private static final double thetaTolerance =
      Units.degreesToRadians(DriveConstants.preciseRotationTolerance);
  private static final double ffMinRadius =
      0.05;
  private static final double ffMaxRadius =
      0.15;

  private final CommandSwerveDrivetrain drive;
  private final Supplier<Pose2d> target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);

  private Translation2d lastSetpointTranslation = new Translation2d();
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;
  private Supplier<SwerveDriveState> driveState;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  public DriveToPoseV2(CommandSwerveDrivetrain drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  public DriveToPoseV2(CommandSwerveDrivetrain drive, Supplier<Pose2d> target, Supplier<SwerveDriveState> robot) {
    this(drive, target);
    this.driveState = robot;
  }

  public DriveToPoseV2(
      CommandSwerveDrivetrain drive,
      Supplier<Pose2d> target,
      Supplier<SwerveDriveState> driveState,
      Supplier<Translation2d> linearFF,
      DoubleSupplier omegaFF) {
    this(drive, target, driveState);
    this.linearFF = linearFF;
    this.omegaFF = omegaFF;
  }

  @Override
  public void initialize() {
        driveController.setP(drivekP);
        driveController.setD(drivekD);
        driveController.setConstraints(
            new TrapezoidProfile.Constraints(driveMaxVelocity, driveMaxAcceleration));
        driveController.setTolerance(driveTolerance);
        thetaController.setP(thetakP);
        thetaController.setD(thetakD);
        thetaController.setConstraints(
            new TrapezoidProfile.Constraints(thetaMaxVelocity, thetaMaxAcceleration));
        thetaController.setTolerance(thetaTolerance);


        Pose2d currentPose = driveState.get().Pose;
        ChassisSpeeds fieldVelocity = drive.getFieldRelativeChassisSpeeds();
        Translation2d linearFieldVelocity =
                new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
        driveController.reset(
                currentPose.getTranslation().getDistance(target.get().getTranslation()),
                Math.min(
                0.0,
                -linearFieldVelocity
                        .rotateBy(
                        target
                                .get()
                                .getTranslation()
                                .minus(currentPose.getTranslation())
                                .getAngle()
                                .unaryMinus())
                        .getX()));
        thetaController.reset(
                currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    // Get current pose and target pose
    Pose2d currentPose = driveState.get().Pose;
    Pose2d targetPose = target.get();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;

    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    GremlinLogger.debugLog("DriveState/PID", driveController.calculate(driveErrorAbs, 0.0));
    GremlinLogger.debugLog("DriveState/FF", driveController.getSetpoint().velocity * ffScaler);
    

    if (currentDistance < driveController.getPositionTolerance()) {
        driveVelocityScalar = 0.0; 
    }
    
    // Adjust PID output when feedforward is low (e.g., close to the target)
    if (ffScaler <= 0.01) { 
        driveVelocityScalar *= 10.0; // Increase PID effect when FF is low
    }  

    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    // Command speeds
    drive.setControl(
        drive.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation())));

    GremlinLogger.debugLog("DriveState/VeloX", driveVelocity.getX());
    GremlinLogger.debugLog("DriveState/VeloY", driveVelocity.getY());
    GremlinLogger.debugLog("DriveState/VeloTheta", thetaVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    drive.driveRobotRelative(new ChassisSpeeds());
    running = false;
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}