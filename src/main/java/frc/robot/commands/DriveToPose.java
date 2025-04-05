package frc.robot.commands;

import static frc.robot.constants.DriveConstants.MAX_ANGULAR_ACCEL_AUTOSCORE;
import static frc.robot.constants.DriveConstants.MAX_ANGULAR_VELOCITY_AUTOSCORE;
import static frc.robot.constants.DriveConstants.MaxSpeed;

import java.util.DoubleSummaryStatistics;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

  /**
   * Default constraints are 90% of max speed, accelerate to full speed in 1/3
   * second
   */
  private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      DriveConstants.MAX_VELO_AUTOSCORE,
      DriveConstants.MAX_ACCEL_AUTOSCORE);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_VELOCITY_AUTOSCORE,
      MAX_ANGULAR_ACCEL_AUTOSCORE);

  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;

  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final Supplier<Pose2d> goalPoseSupplier;
  private final DoubleSupplier xFF;
  private final DoubleSupplier yFF;

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
      DoubleSupplier xFF,
      DoubleSupplier yFF) {
    this(drivetrainSubsystem, poseProvider, goalPoseSup, DEFAULT_XY_CONSTRAINTS, DEFAULT_OMEGA_CONSTRAINTS, xFF, yFF);
  }

  public DriveToPose(
      CommandSwerveDrivetrain drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      Supplier<Pose2d> goalPoseSup,
      TrapezoidProfile.Constraints xyConstraints,
      TrapezoidProfile.Constraints omegaConstraints) {
    
        this(
          drivetrainSubsystem, 
          poseProvider, 
          goalPoseSup, 
          DEFAULT_XY_CONSTRAINTS, 
          DEFAULT_OMEGA_CONSTRAINTS,
          () -> 0,
          () -> 0);
  }

  public DriveToPose(
      CommandSwerveDrivetrain drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      Supplier<Pose2d> goalPoseSup,
      TrapezoidProfile.Constraints xyConstraints,
      TrapezoidProfile.Constraints omegaConstraints,
      DoubleSupplier xFF,
      DoubleSupplier yFF) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.goalPoseSupplier = goalPoseSup;
    this.xFF = xFF;
    this.yFF = yFF;

    xController = new PIDController(DriveConstants.preciseTranslationkP, DriveConstants.preciseTranslationkI,
        DriveConstants.preciseTranslationkD);
    yController = new PIDController(DriveConstants.preciseTranslationkP, DriveConstants.preciseTranslationkI,
        DriveConstants.preciseTranslationkD);
    xController.setTolerance(DriveConstants.preciseTranslationTolerance);
    yController.setTolerance(DriveConstants.preciseTranslationTolerance);
    thetaController = new ProfiledPIDController(DriveConstants.preciseRotationkP, DriveConstants.preciseRotationkI,
        DriveConstants.preciseRotationkD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(DriveConstants.preciseRotationTolerance));

    resetPIDControllers();
    
    if(goalPoseSupplier.get() == null){
      try {
        throw new Exception("How the hell is this null");
      } catch (Exception e) {
        e.printStackTrace();
      }
    } else {
      goalPose = AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(goalPoseSupplier.get()) : goalPoseSupplier.get();
    }

    addRequirements(this.drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    resetPIDControllers();
    
    if(goalPoseSupplier.get() == null){
      try {
        throw new Exception("How the hell is this null");
      } catch (Exception e) {
        e.printStackTrace();
      }
    } else {
      goalPose = AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(goalPoseSupplier.get()) : goalPoseSupplier.get();
    }

    thetaController.setGoal(goalPose.getRotation().getRadians());
    xController.setSetpoint(goalPose.getX());
    yController.setSetpoint(goalPose.getY());

    targetPosePublisher.set(goalPose);
  }

  public boolean atGoal() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atGoal() 
      && drivetrainSubsystem.getState().Speeds.vxMetersPerSecond < 0.05
      && drivetrainSubsystem.getState().Speeds.vyMetersPerSecond < 0.05
      && drivetrainSubsystem.getState().Speeds.omegaRadiansPerSecond < Units.degreesToRadians(5);
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    if(goalPose == null){
      initialize();
    }

    Pose2d robotPose = poseProvider.get();
    // Drive to the goal
    double xSpeed = xController.calculate(robotPose.getX());
    if (xController.atSetpoint()) {
      xSpeed = 0;
    }

    double ySpeed = yController.calculate(robotPose.getY());
    if (yController.atSetpoint()) {
      ySpeed = 0;
    }

    double omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    xSpeed += xFF.getAsDouble(); //Add user input from joysticks as a feedforward
    ySpeed += yFF.getAsDouble(); //Should already be in given in terms of desired speed

    drivetrainSubsystem.setControl(DriveConstants.APPLY_FIELD_SPEEDS
        .withSpeeds(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed)));
      
    SmartDashboard.putNumber("DriveState/X", xSpeed);
    SmartDashboard.putNumber("DriveState/Y", ySpeed);
    SmartDashboard.putNumber("DriveState/Theta", omegaSpeed);
    SmartDashboard.putNumber("DriveState/GoalX", xController.getSetpoint());
    SmartDashboard.putNumber("DriveState/GoalY", yController.getSetpoint());
    SmartDashboard.putNumber("DriveState/GoalTheta", thetaController.getGoal().position);
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