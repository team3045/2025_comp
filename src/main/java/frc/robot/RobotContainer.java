// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState.DriveState;
import frc.robot.commands.AutoScoreFactory;
import frc.robot.commands.IntakeSequenceFactory;
import frc.robot.commons.GremlinPS4Controller;
import frc.robot.commons.GremlinUtil;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.vision.apriltag.GremlinApriltagVision;
import frc.robot.vision.apriltag.VisionConstants;

import static frc.robot.constants.DriveConstants.MaxSpeed;
import static frc.robot.constants.DriveConstants.MaxAngularRate;;


public class RobotContainer {
    public static final RobotState M_ROBOT_STATE = RobotState.getRobotState();
     private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final GremlinPS4Controller joystick = new GremlinPS4Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final GremlinApriltagVision vision = new GremlinApriltagVision(VisionConstants.cameras,
        () -> drivetrain.getState().Pose, 
        VisionConstants.limelights,
        (drivetrain::addVisionMeasurements));
    public final ElevatorPivot elevatorPivot = new ElevatorPivot();
    public final Claw claw = new Claw();

    /*Auto Score Stuff */
    public final AutoScoreFactory autoScoreFactory = new AutoScoreFactory(drivetrain, elevatorPivot, claw);

    /* intake sequence */
    public final IntakeSequenceFactory intakeSequenceFactory = new IntakeSequenceFactory(drivetrain, elevatorPivot, claw);

    /*Triggers */
    private final Trigger scoringState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.AUTOSCORE);
    private final Trigger algeaState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.ALGEA);
    private final Trigger intakeState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.INTAKE);
    private final Trigger disableGlobalEstimation = (scoringState.or(algeaState)).and(() -> drivetrain.withinDistanceOfReef(FieldConstants.reefDistanceTolerance));


    public RobotContainer() {
        DogLog.setOptions(new DogLogOptions());
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();


        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                DriveConstants.drive.withVelocityX(GremlinUtil.squareDriverInput(-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(GremlinUtil.squareDriverInput(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(GremlinUtil.squareDriverInput(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.options().and(joystick.cross()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.share().and(joystick.cross()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.R1().onTrue(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.AUTOSCORE)).unless(claw.hasCoral));
        joystick.R1().onFalse(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)));
        
        scoringState.whileTrue(
            autoScoreFactory.pathFindWithApriltagFeeback(VisionConstants.limelights[0], VisionConstants.limelights[1]) //righ and left
            .alongWith(autoScoreFactory.setElevatorHeight())
            .andThen(claw.clawOutake())
            .andThen(Commands.waitSeconds(0.4))
            .andThen(drivetrain.driveBack())
            .finallyDo(() -> {
                M_ROBOT_STATE.setDriveState(DriveState.TELEOP);
                }) //REDENDUNCY TO ALWAYS SET BACK TO TELEOP AFTER SCORE
            );

        scoringState.onFalse(
            elevatorPivot.stowArm().alongWith(claw.stop())); //STOW ARM AND STOP CLAW AFTER SCORING
        
        disableGlobalEstimation.onTrue(Commands.runOnce(() -> vision.setRejectAllUpdates(true)));
        disableGlobalEstimation.onFalse(Commands.runOnce(() -> vision.setRejectAllUpdates(false)));

        joystick.L1().onTrue(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.ALGEA)));
        joystick.L1().onFalse(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)));

        algeaState.whileTrue(
            autoScoreFactory.getAlgeaRemoveCommand(VisionConstants.limelights[0])
            .finallyDo(() -> {
                M_ROBOT_STATE.setDriveState(DriveState.TELEOP);
                }) //REDENDUNCY TO ALWAYS SET BACK TO TELEOP AFTER REMOVAL
        );

        joystick.povDown().onTrue(elevatorPivot.zeroHeight());
        joystick.square().onTrue(elevatorPivot.stowArm());
                
        // joystick.circle().whileTrue(elevatorPivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // joystick.cross().whileTrue(elevatorPivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // joystick.square().whileTrue(elevatorPivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // joystick.triangle().whileTrue(elevatorPivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        joystick.R2().onTrue(
            new ConditionalCommand(
                Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.INTAKE)), 
                Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)), 
                intakeState.negate().and(claw.hasCoral.negate()))
        );

        intakeState.whileTrue(
            drivetrain.driveFacingIntake(
                () -> GremlinUtil.squareDriverInput(-joystick.getLeftY()) * MaxSpeed , 
                () -> GremlinUtil.squareDriverInput(-joystick.getLeftX()) * MaxSpeed).alongWith(
            elevatorPivot.goToIntake()
            .andThen(claw.clawIntake()
                .andThen(Commands.waitUntil(claw.hasCoral))
                .andThen(claw.slowIntake())
                .andThen(Commands.waitUntil(claw.hasCoral.negate()))
                .andThen(claw.slowBackup())
                .andThen(Commands.waitUntil(claw.hasCoral))
                .finallyDo(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)))
        ));


        intakeState.onFalse(claw.stop());
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
