package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveToPoseV2;
import frc.robot.commands.DriveToPoseV3;
import frc.robot.commands.DynamicPathfindCommand;
import frc.robot.commons.GremlinAutoBuilder;
import frc.robot.commons.TimestampedVisionUpdate;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.vision.apriltag.VisionConstants;

import static frc.robot.constants.DriveConstants.*;
import static frc.robot.constants.FieldConstants.blueReefCenter;
import static frc.robot.constants.FieldConstants.redReefCenter;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    Pigeon2 kPigeon2 = new Pigeon2(TunerConstants.kPigeonId, TunerConstants.kCANBus);

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    public final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* Swerve Setpoint Generator */
    private SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;
    private RobotConfig config;

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));


    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        configurePathPlannerLogging();
        configureSetpointGenerator();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        configurePathPlannerLogging();
        configureSetpointGenerator();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        configurePathPlannerLogging();
        configureSetpointGenerator();
    }

    private void configureAutoBuilder() {
        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    pathFollowingController,
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );

            GremlinAutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    pathFollowingController,
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );

        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }

        // Set the PID gains for the heading controller for facing angle commands
        facingAngle.HeadingController.setPID(headingP, headingI, headingD);
    }

    private void configureSetpointGenerator() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            setpointGenerator = new SwerveSetpointGenerator(
                    config,
                    MAX_STEER_VELOCITY);

            // Initialize the previous setpoint to the robot's current speeds & module
            // states
            ChassisSpeeds currentSpeeds = getState().Speeds; // Method to get current robot-relative chassis speeds
            SwerveModuleState[] currentStates = getState().ModuleStates; // Method to get the current swerve module
                                                                         // states
            previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates,
                    DriveFeedforwards.zeros(config.numModules));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public double[] getWheelPositionsRadians() {
        double[] positionRads = new double[4];
        int i = 0;
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : getModules()) {
            double positionRotations = module.getDriveMotor().getPosition().getValueAsDouble()
                    / TunerConstants.kDriveGearRatio;
            positionRads[i] = Units.rotationsToRadians(positionRotations);
            i++;
        }
        return positionRads;
    }

    public void turnAtRotationalRate(double radsPerSecond) {
        setControl(m_rotationCharacterization.withRotationalRate(radsPerSecond));
    }

    public double getRawHeadingRadians() {
        return getState().RawHeading.getRadians();
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds(){
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            getState().Speeds, getState().Pose.getRotation());
    }

    public static final StructArrayPublisher<Pose2d> TELEOP_TRAJECTORY_PUBLISHER = NetworkTableInstance.getDefault()
            .getStructArrayTopic(DRIVE_LOG_PATH + "Trajectory/Path", Pose2d.struct).publish();
    public static final StructPublisher<Pose2d> TARGET_POSE_PUBLISHER = NetworkTableInstance.getDefault()
            .getStructTopic(DRIVE_LOG_PATH + "Trajectory/Target Pose", Pose2d.struct).publish();

    private void configurePathPlannerLogging() {
        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            TARGET_POSE_PUBLISHER.set(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            TELEOP_TRAJECTORY_PUBLISHER.set(poses.toArray(Pose2d[]::new));
        });
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Adds a list of vision updates along with their timestamps and standard
     * deviations
     * 
     * @param updates the vision updates to add
     */
    public void addVisionMeasurements(List<TimestampedVisionUpdate> updates) {
        for (int i = 0; i < updates.size(); i++) {
            addVisionMeasurement(updates.get(i).pose(), Utils.fpgaToCurrentTime(updates.get(i).timestamp()),
                    updates.get(i).stdDevs());
        }
    }

    /**
     * This method will take in desired robot-relative chassis speeds,
     * generate a swerve setpoint, then set the target state for each module
     *
     * @param speeds The desired robot-relative speeds
     */
    public SwerveRequest.ApplyRobotSpeeds driveRobotRelative(ChassisSpeeds speeds) {
        // Note: it is important to not discretize speeds before or after
        // using the setpoint generator, as it will discretize them for you
        previousSetpoint = setpointGenerator.generateSetpoint(
                previousSetpoint, // The previous setpoint
                speeds, // The desired target speeds
                0.02 // The loop time of the robot code, in seconds
        );
        return m_pathApplyRobotSpeeds
                .withSpeeds(previousSetpoint.robotRelativeSpeeds())
                .withWheelForceFeedforwardsX(previousSetpoint.feedforwards().robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(previousSetpoint.feedforwards().robotRelativeForcesYNewtons()); // Method
                                                                                                             // that
                                                                                                             // will
                                                                                                             // drive
                                                                                                             // the
                                                                                                             // robot
                                                                                                             // given
                                                                                                             // target
                                                                                                             // module
                                                                                                             // states
    }

    public Command driveFacingAngle(Supplier<Rotation2d> angleSupplier, DoubleSupplier xSpeeds,
            DoubleSupplier ySpeeds) {
        return this.applyRequest(() -> facingAngle.withTargetDirection(angleSupplier.get())
                .withVelocityX(xSpeeds.getAsDouble())
                .withVelocityY(ySpeeds.getAsDouble()));
    }

    public Command driveFacingIntake(DoubleSupplier xSpeeds, DoubleSupplier ySpeeds) {
        Supplier<Rotation2d> angleSupplier = () -> {
            Rotation2d returnAngle;
            if (getState().Pose.getY() > (FieldConstants.compFieldWidth / 2)) {
                returnAngle = Rotation2d.fromDegrees(125);
            } else {
                returnAngle = Rotation2d.fromDegrees(235);
            }

            return AutoBuilder.shouldFlip() ? returnAngle.times(-1).plus(Rotation2d.k180deg)
                    : returnAngle.minus(Rotation2d.k180deg);
        };

        return driveFacingAngle(angleSupplier, xSpeeds, ySpeeds);
    }

    public Command driveFacingBarge(DoubleSupplier xSpeeds, DoubleSupplier ySpeeds){
        Supplier<Rotation2d> angSupplier = () -> Rotation2d.k180deg;

        return driveFacingAngle(angSupplier, xSpeeds, ySpeeds);
    }

    public Command driveFacingProcessor(DoubleSupplier xSpeeds, DoubleSupplier ySpeeds) {
        Supplier<Rotation2d> angSupplier = () -> {
            return AutoBuilder.shouldFlip()
                    ? FlippingUtil.flipFieldRotation(FieldConstants.Processor.centerFace.getRotation())
                    : FieldConstants.Processor.centerFace.getRotation().times(-1);
        };

        return driveFacingAngle(angSupplier, xSpeeds, ySpeeds);
    }

    public Command driveFacingAlgea(DoubleSupplier xSpeeds, DoubleSupplier ySpeeds) {
        Supplier<Rotation2d> angleSupplier = () -> {
            List<Pose2d> poseList = AutoBuilder.shouldFlip() ? FieldConstants.flippedAlgeaPoses
                    : FieldConstants.algeaPoses;

            Pose2d closest = poseList.get(0);
            int closestNum = 0;

            for (int i = 1; i < poseList.size(); i++) {
                if (poseList.get(i).getTranslation().getDistance(getState().Pose.getTranslation()) < closest
                        .getTranslation().getDistance(getState().Pose.getTranslation())) {
                    closest = poseList.get(i);
                    closestNum = i;
                }
            }

            return FieldConstants.algeaPoses.get(closestNum).getRotation();
        };

        return driveFacingAngle(angleSupplier, xSpeeds, ySpeeds);
    }

    public Command driveFacingTrough(DoubleSupplier xSpeeds, DoubleSupplier ySpeeds) {
        Supplier<Rotation2d> angleSupplier = () -> {
            List<Pose2d> poseList = AutoBuilder.shouldFlip() ? FieldConstants.flippedAlgeaPoses
                    : FieldConstants.algeaPoses;

            Pose2d closest = poseList.get(0);
            int closestNum = 0;

            for (int i = 1; i < poseList.size(); i++) {
                if (poseList.get(i).getTranslation().getDistance(getState().Pose.getTranslation()) < closest
                        .getTranslation().getDistance(getState().Pose.getTranslation())) {
                    closest = poseList.get(i);
                    closestNum = i;
                }
            }

            return FieldConstants.algeaPoses.get(closestNum).getRotation();
        };

        return driveFacingAngle(angleSupplier, xSpeeds, ySpeeds);
    }

    public Command preciseTargetPose(Supplier<Pose2d> targetPose) {
        return new DriveToPose(this, () -> getState().Pose, targetPose);
        //return new DriveToPoseV2(this, targetPose, () -> getState());
        // return new DriveToPoseV3(
        //     this, () -> getState(), targetPose);
    }

    public Command targetPoseWithJoystick(Supplier<Pose2d> targetPose){
        return new DriveToPoseV2(
            this, 
            targetPose,
            () -> getState());
    }
    
    /**
     * Returns a command that will drive robot to supplied targetPose using
     * Pathplanner Pathfind
     * 
     * @param targetPoseSup         A supplier of the desired pose to drive to
     * @param desiredEndVelocitySup A supplier of the desired end velocity
     * @return A command to drive robot to desired Pose
     */
    public Command pathFindToPose(Supplier<Pose2d> targetPoseSup, DoubleSupplier desiredEndVelocitySup) {
        return new DynamicPathfindCommand(targetPoseSup, desiredEndVelocitySup, pathFollowingConstraints, this);
    }

    public boolean withinDistanceOfReef(double distance) {
        return getState().Pose.getTranslation().getDistance(
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? blueReefCenter
                        : redReefCenter) < distance;
    }

    public Command driveBack() {
        return applyRequest(() -> driveBack).withTimeout(0.2);
    }

    public Command driveForward() {
        return applyRequest(() -> driveForward).withTimeout(0.2);
    }

    public Command driveBackwardBarge() {
        return applyRequest(() -> driveBack).withTimeout(0.8).andThen(Commands.runOnce(() -> setControl(brake)));
    }


    public Command driveBackAlgea() {
        return applyRequest(() -> driveBack).withTimeout(0.5);
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public Command maxSpeedTest(){
        return applyRequest(() -> m_translationCharacterization.withVolts(12));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        VisionConstants.limelights[0].setRobotHeading(getState().Pose.getRotation().getDegrees());
        VisionConstants.limelights[1].setRobotHeading(getState().Pose.getRotation().getDegrees());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}