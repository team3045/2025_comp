// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GremlinRobotState.DriveState;
import frc.robot.commands.AutoScoreFactory;
import frc.robot.commands.IntakeSequenceFactory;
import frc.robot.commons.GremlinLogger;
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

import javax.naming.Name;

import static frc.robot.constants.DriveConstants.MaxAngularRate;;


public class RobotContainer {
    public static final GremlinRobotState M_ROBOT_STATE = GremlinRobotState.getRobotState();
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
    public final Trigger isAuton = new Trigger(() -> DriverStation.isAutonomous());
    public final Trigger scoringState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.AUTOSCORE);
    public final Trigger algeaState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.ALGEA);
    public final Trigger intakeState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.INTAKE);
    public final Trigger teleopState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.TELEOP);
    public final Trigger disableGlobalEstimation = (scoringState.or(algeaState)).and(() -> drivetrain.withinDistanceOfReef(FieldConstants.reefDistanceTolerance));

    public RobotContainer() {
        DogLog.setOptions(new DogLogOptions()
            .withNtPublish(false)
            .withCaptureNt(GremlinLogger.isDebug())
            .withCaptureConsole(GremlinLogger.isDebug())
            .withLogExtras(GremlinLogger.isDebug()));

        registerPathPlannerCommands();
        configureAutoTriggers();

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

        joystick.R1().onTrue(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.AUTOSCORE)));
        joystick.R1().onFalse(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)));
        
        scoringState.whileTrue(autoScoreFactory.fullAutoScoreCommand());

        scoringState.negate().and(intakeState.negate()).and(algeaState.negate()).and(isAuton.negate()).whileTrue(
            elevatorPivot.stowArm().alongWith(claw.stop())); //STOW ARM AND STOP CLAW AFTER SCORING
        
        disableGlobalEstimation.onTrue(Commands.runOnce(() -> vision.setRejectAllUpdates(true)));
        disableGlobalEstimation.onFalse(Commands.runOnce(() -> vision.setRejectAllUpdates(false)));

        joystick.L1().onTrue(
            new ConditionalCommand(
                Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.ALGEA)), 
                Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)), 
                algeaState.negate()));

        algeaState.whileTrue(
            autoScoreFactory.getAlgeaRemoveCommand(
                VisionConstants.limelights[0],
                () -> GremlinUtil.squareDriverInput(-joystick.getLeftY()) * MaxSpeed,
                () -> GremlinUtil.squareDriverInput(-joystick.getLeftX()) * MaxSpeed)
            .finallyDo(() -> {
                M_ROBOT_STATE.setDriveState(DriveState.TELEOP);
                }) //REDENDUNCY TO ALWAYS SET BACK TO TELEOP AFTER REMOVAL
        );

        algeaState.onFalse(
            elevatorPivot.stowArm().alongWith(claw.fullHold()));

        joystick.povDown().onTrue(elevatorPivot.zeroHeight());
        joystick.square().onTrue(elevatorPivot.stowArm());

        joystick.R2().onTrue(
            new ConditionalCommand(
                Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.INTAKE)), 
                Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)), 
                intakeState.negate())
        );

        intakeState.whileTrue(
            drivetrain.driveFacingIntake(
                () -> GremlinUtil.squareDriverInput(-joystick.getLeftY()) * MaxSpeed , 
                () -> GremlinUtil.squareDriverInput(-joystick.getLeftX()) * MaxSpeed).alongWith(
            elevatorPivot.goToIntake()
            .andThen(claw.fullIntake()
                .andThen(Commands.waitUntil(claw.hasCoral))
                .andThen(claw.slowIntake())
                .andThen(Commands.waitUntil(claw.hasCoral.negate()))
                .andThen(claw.slowBackup())
                .andThen(Commands.waitUntil(claw.hasCoral))
                .andThen(claw.driveBack())
                .finallyDo(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)))
        ));



        intakeState.onFalse(claw.fullHold());

        joystick.L2().OnPressTwice(
            drivetrain.driveFacingProcessor(
                () -> GremlinUtil.squareDriverInput(-joystick.getLeftY()) * MaxSpeed , 
                () -> GremlinUtil.squareDriverInput(-joystick.getLeftX()) * MaxSpeed)
            .alongWith(elevatorPivot.goToProcessor()),
            claw.algeaOuttake()
                .andThen(Commands.waitUntil(ElevatorPivot.hasAlgea.negate()))
                .andThen(claw.hold())
                .andThen(drivetrain.driveBack())
        );

        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void registerPathPlannerCommands(){
        NamedCommands.registerCommand("intake", 
            elevatorPivot.goToIntake()
                .andThen(claw.fullIntake()
                .andThen(Commands.waitUntil(claw.hasCoral))
                .andThen(claw.slowIntake())
                .andThen(Commands.waitUntil(claw.hasCoral.negate()))
                .andThen(claw.slowBackup())
                .andThen(Commands.waitUntil(claw.hasCoral))));

        NamedCommands.registerCommand("waitUntilScored", 
            Commands.waitUntil(claw.hasCoral.negate()).andThen(Commands.waitSeconds(0.3)));
        
        NamedCommands.registerCommand("waitUntilIntake", 
            Commands.waitUntil(claw.hasCoral).withTimeout(2).withName("waitUntilIntake"));

        NamedCommands.registerCommand("ScoreCoral",
            claw.clawOutake()
            .andThen(Commands.waitSeconds(0.4)).withName("Score Coral"));
        
        NamedCommands.registerCommand("StartScoreF",
            autoScoreFactory.AutonomousPeriodAutoScore(() -> 3,() -> 6,
            VisionConstants.limelights[1], 
            VisionConstants.limelights[0]).withName("StartScoreF"));
    }

    public void configureAutoTriggers(){
        new EventTrigger("StartScoreF").onTrue(
            autoScoreFactory.AutonomousPeriodAutoScore(() -> 3,() -> 6,
            VisionConstants.limelights[1], 
            VisionConstants.limelights[0]));

        new EventTrigger("StartScoreE").onTrue(
            autoScoreFactory.AutonomousPeriodAutoScore(() -> 3, () -> 5, 
            VisionConstants.limelights[1], 
            VisionConstants.limelights[0]));

        new EventTrigger("StartIntake").onTrue(
            elevatorPivot.goToIntake()
                .andThen(claw.fullIntake()
                .andThen(Commands.waitUntil(claw.hasCoral))
                .andThen(claw.slowIntake())
                .andThen(Commands.waitUntil(claw.hasCoral.negate()))
                .andThen(claw.driveBack())
                .andThen(Commands.waitUntil(claw.hasCoral)))
        );
        
    }

}
