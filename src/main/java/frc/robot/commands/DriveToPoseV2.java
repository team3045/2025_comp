package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.GremlinLogger;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Drives to a specified pose.
 */
public class DriveToPoseV2 extends Command {
    private final ProfiledPIDController driveController = new ProfiledPIDController(
            DriveConstants.preciseTranslationkP, DriveConstants.preciseTranslationkI, DriveConstants.preciseTranslationkD, 
            new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            DriveConstants.preciseRotationkP, DriveConstants.preciseRotationkI, DriveConstants.preciseRotationkD, 
            new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private CommandSwerveDrivetrain driveSubsystem;
    private Supplier<Pose2d> targetPoseSupplier;
    private Supplier<SwerveDriveState> driveStateSupplier;

    private Translation2d lastSetpointTranslation;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.2, ffMaxRadius = 0.8;

    public DriveToPoseV2(CommandSwerveDrivetrain driveSubsystem, Supplier<SwerveDriveState> stateSupplier, Supplier<Pose2d> targetPoseSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.driveStateSupplier = stateSupplier;
        this.targetPoseSupplier = targetPoseSupplier;
        addRequirements(driveSubsystem);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = driveStateSupplier.get().Pose;
        driveController.reset(
                currentPose.getTranslation().getDistance(targetPoseSupplier.get().getTranslation()),
                Math.min(
                        0.0,
                        -new Translation2d(driveSubsystem.getFieldRelativeChassisSpeeds().vxMetersPerSecond,
                        driveSubsystem.getFieldRelativeChassisSpeeds().vyMetersPerSecond)
                                .rotateBy(
                                        targetPoseSupplier
                                                .get()
                                                .getTranslation()
                                                .minus(driveStateSupplier.get().Pose.getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(currentPose.getRotation().getRadians(),
                driveStateSupplier.get().Speeds.omegaRadiansPerSecond);
        lastSetpointTranslation = driveStateSupplier.get().Pose.getTranslation();
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveStateSupplier.get().Pose;
        Pose2d targetPose = targetPoseSupplier.get();

        GremlinLogger.debugLog("DriveToPose/currentPose", currentPose);
        GremlinLogger.debugLog("DriveToPose/targetPose", targetPose);

        double currentDistance = currentPose.getTranslation().getDistance(targetPoseSupplier.get().getTranslation());
        double ffScaler = MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);
        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance())
            driveVelocityScalar = 0.0;
        lastSetpointTranslation = new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(
                        GeomUtil.transform2dFromTranslation(
                                new Translation2d(driveController.getSetpoint().position, 0.0)))
                .getTranslation();

        // Calculate theta speed
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance())
            thetaVelocity = 0.0;

        // Command speeds
        var driveVelocity = GeomUtil
                .pose2dFromRotation(currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(GeomUtil.transform2dFromTranslation(new Translation2d(driveVelocityScalar, 0.0)))
                .getTranslation();
        driveSubsystem.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setControl(DriveConstants.APPLY_FIELD_SPEEDS.withSpeeds(new ChassisSpeeds()));
    }

    @Override
    public boolean isFinished() {
        return targetPoseSupplier.get().equals(null) || (driveController.atGoal() && thetaController.atGoal());
    }
}