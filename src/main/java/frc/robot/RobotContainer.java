// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GremlinRobotState.DriveState;
import frc.robot.commands.AutoScoreFactory;
import frc.robot.commons.GremlinAutoBuilder;
import frc.robot.commons.GremlinLogger;
import frc.robot.commons.GremlinPS4Controller;
import frc.robot.commons.GremlinUtil;
import frc.robot.constants.AutoScoreConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ElevatorPivotConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.vision.VisionSubsystem;

import static frc.robot.constants.DriveConstants.MaxSpeed;
import static frc.robot.constants.DriveConstants.drive;

import java.util.function.Supplier;

import static frc.robot.constants.DriveConstants.MaxAngularRate;;


public class RobotContainer {
    public static final GremlinRobotState M_ROBOT_STATE = GremlinRobotState.getRobotState();
    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final GremlinPS4Controller joystick = new GremlinPS4Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorPivot elevatorPivot = new ElevatorPivot();
    public final Claw claw = new Claw();
    private final CommandGenericHID buttonBoard = new CommandGenericHID(1);

    // private IntegerSubscriber poleNumberSub = NetworkTableInstance.getDefault().getTable("Scoring Location")
    //   .getIntegerTopic("Pole").subscribe(0);
    // private IntegerSubscriber heightSub = NetworkTableInstance.getDefault().getTable("Scoring Location")
    //   .getIntegerTopic("Height").subscribe(0);
    // private Supplier<Integer> poleNumSub = () -> {
    //     if (DriverStation.getAlliance().get() == Alliance.Blue)
    //     return (int) poleNumberSub.get();
    //     else
    //     return (int) poleNumberSub.get() + 12;
    // };

    // public static final IntegerPublisher heightPublisher = NetworkTableInstance.getDefault().getTable("Scoring Location")
    //   .getIntegerTopic("Height").publish();

    

    /*Auto Score Stuff */
    
    // public final AutoScoreFactory autoScore = new AutoScoreFactory(drivetrain, elevatorPivot, claw, () -> poleNumSub.get(), () -> (int) heightSub.get());
    public final AutoScoreFactory autoScore = new AutoScoreFactory(drivetrain, elevatorPivot, claw, () -> elevatorPivot.scorePole, () -> elevatorPivot.scoreHeight);

    public final VisionSubsystem vision = new VisionSubsystem(drivetrain);

    /*Triggers */
    public final Trigger isAuton = new Trigger(() -> DriverStation.isAutonomous());
    public final Trigger scoringState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.AUTOSCORE);
    public final Trigger algeaState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.ALGEA);
    public final Trigger intakeState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.INTAKE);
    public final Trigger teleopState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.TELEOP);
    public final Trigger processorState = new Trigger(() -> M_ROBOT_STATE.getDriveState() == DriveState.PROCESSOR);
    public final Trigger disableGlobalEstimation = (scoringState.or(algeaState).or(isAuton)).and(() -> drivetrain.withinDistanceOfReef(FieldConstants.reefDistanceTolerance)).debounce(0.4,DebounceType.kFalling);

    public RobotContainer() {
        DogLog.setOptions(new DogLogOptions()
            .withNtPublish(false)
            .withCaptureNt(GremlinLogger.isDebug())
            .withCaptureConsole(GremlinLogger.isDebug())
            .withLogExtras(GremlinLogger.isDebug()));

        registerPathPlannerCommands();
        configureAutoTriggers();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = GremlinAutoBuilder.buildAutoChooser();
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
        joystick.options().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.R3().onTrue(claw.clawOutake());
        
        // scoringState.whileTrue(autoScoreFactory.fullAutoScoreCommand());

        teleopState.and(isAuton.negate()).whileTrue(
            elevatorPivot.stowArm().alongWith(claw.stop())); //STOW ARM AND STOP CLAW AFTER SCORING

        joystick.L1().onTrue(elevatorPivot.runOnce(() -> {elevatorPivot.isRight = false; elevatorPivot.scorePole = getPoleNum();}).andThen(autoScore.autoScore().onlyWhile(() -> joystick.L1().getAsBoolean())));
        joystick.R1().onTrue(elevatorPivot.runOnce(() -> {elevatorPivot.isRight = true; elevatorPivot.scorePole = getPoleNum();}).andThen(autoScore.autoScore().onlyWhile(() -> joystick.R1().getAsBoolean())));


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

        joystick.L2().onTrue(
            Commands.either(
                Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.PROCESSOR)), 
                Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)),
                processorState.negate()
            ));

        processorState.onTrue(drivetrain.driveFacingProcessor(
                () -> GremlinUtil.squareDriverInput(-joystick.getLeftY()) * MaxSpeed , 
                () -> GremlinUtil.squareDriverInput(-joystick.getLeftX()) * MaxSpeed)
            .alongWith(elevatorPivot.goToProcessor()));
        
        processorState.onFalse(claw.algeaOuttake()
            .andThen(Commands.waitUntil(ElevatorPivot.hasAlgea.negate()))
            .andThen(claw.hold())
            .andThen(drivetrain.driveBack()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // joystick.share().whileTrue(drivetrain.maxSpeedTest());
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
        
        // NamedCommands.registerCommand("StartScoreF",
        //     autoScore.autoScoreCustom(3,6).withName("StartScoreF"));

        // NamedCommands.registerCommand("StartScoreE",
        //     autoScore.autoScoreCustom(3,5).withName("StartScoreE"));

        // NamedCommands.registerCommand("StartScoreD",
        //     autoScore.autoScoreCustom(3,4).withName("StartScoreE"));

        // NamedCommands.registerCommand("StartScoreC",
        //     autoScore.autoScoreCustom(3,3).withName("StartScoreE"));
        
        NamedCommands.registerCommand("StartIntake", 
            elevatorPivot.goToIntake()
                .andThen(claw.fullIntake()
                .andThen(Commands.waitUntil(claw.hasCoral))
                .andThen(claw.slowIntake())
                .andThen(Commands.waitUntil(claw.hasCoral.negate()))
                .andThen(claw.slowBackup())
                .andThen(Commands.waitUntil(claw.hasCoral))
                .andThen(claw.fullHold())));

        NamedCommands.registerCommand("StowArm", 
            elevatorPivot.stowArm().alongWith(claw.hold())
        );

        NamedCommands.registerCommand("IntakeAlgae", 
            elevatorPivot.goToPosition(
                () -> ElevatorPivotConstants.HeightPositions.LOW_ALGEA.getHeight(), 
                () -> ElevatorPivotConstants.AnglePositions.LOW_ALGEA.getAngle())
            .alongWith(claw.algeaIntake())
            .until(ElevatorPivot.hasAlgea));

        NamedCommands.registerCommand("ProcArm",
            elevatorPivot.goToProcessor()
        );

        NamedCommands.registerCommand("AlgaeOut", 
            claw.algeaOuttake()
        );
    }   

    public void configureAutoTriggers(){
        // new EventTrigger("StartScoreF").onTrue(
        //     autoScore.autoScoreCustom(3,6));

        // new EventTrigger("StartScoreE").onTrue(
        //     autoScore.autoScoreCustom(3, 5));

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
    public void configButtonBoard(){
        buttonBoard.button(1).onTrue(elevatorPivot.runOnce(() -> {
            elevatorPivot.scoreHeight = 3;
        }));
        buttonBoard.button(2).onTrue(elevatorPivot.runOnce(() -> {
            elevatorPivot.scoreHeight = 2;
        }));
        buttonBoard.button(3).onTrue(elevatorPivot.runOnce(() -> {
            elevatorPivot.scoreHeight = 1;
        }));
        buttonBoard.button(7).onTrue(elevatorPivot.runOnce(() -> {
            elevatorPivot.scoreHeight = 3;
        }));
        buttonBoard.button(8).onTrue(elevatorPivot.runOnce(() -> {
            elevatorPivot.scoreHeight = 2;
        }));
        buttonBoard.button(9).onTrue(elevatorPivot.runOnce(() -> {
            elevatorPivot.scoreHeight = 1;
        }));
    }

    public int getPoleNum() {
        int poleNum = 1;
        double dist = 100000;
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            for (int i = 0; i <= 5; i ++) {
                double distNew = drivetrain.getState().Pose.getTranslation().getDistance(getMedian(AutoScoreConstants.kScorePoseMap.get(2 * i + 1).getTranslation(), AutoScoreConstants.kScorePoseMap.get(2 * i + 2).getTranslation()));
                if (distNew < dist) {
                    dist = distNew;
                    if (elevatorPivot.isRight) {
                        poleNum = 2 * i + 2;
                    } else {
                        poleNum = 2 * i + 1;
                    }
                }
            }
        } else {
            for (int i = 6; i <= 11; i ++) {
                double distNew = drivetrain.getState().Pose.getTranslation().getDistance(getMedian(AutoScoreConstants.kScorePoseMap.get(2 * i + 1).getTranslation(), AutoScoreConstants.kScorePoseMap.get(2 * i + 2).getTranslation()));
                if (distNew < dist) {
                    dist = distNew;
                    if (elevatorPivot.isRight) {
                        poleNum = 2 * i + 2;
                    } else {
                        poleNum = 2 * i + 1;
                    }
                }
            }
        }
        SmartDashboard.putNumber("PoleNum", poleNum);
        return poleNum;
    }

    public Translation2d getMedian(Translation2d pose1, Translation2d pose2) {
        return pose1.interpolate(pose2, 0.5);
    }
}
