// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GremlinRobotState.DriveState;
import frc.robot.commands.AutoScoreFactory;
import frc.robot.commands.IntakeSequenceFactory;
import frc.robot.commons.GremlinPS4Controller;
import frc.robot.commons.GremlinUtil;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
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
    public final Climber climber = new Climber();

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

        joystick.R1().onTrue(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.AUTOSCORE)));
        joystick.R1().onFalse(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)));
        
        scoringState.whileTrue(autoScoreFactory.fullAutoScoreCommand());

        scoringState.onFalse(
            elevatorPivot.stowArm().alongWith(claw.stop())); //STOW ARM AND STOP CLAW AFTER SCORING
        
        disableGlobalEstimation.onTrue(Commands.runOnce(() -> vision.setRejectAllUpdates(true)));
        disableGlobalEstimation.onFalse(Commands.runOnce(() -> vision.setRejectAllUpdates(false)));

        joystick.L1().onTrue(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.ALGEA)).unless(ElevatorPivot.hasAlgea));
        joystick.L1().onFalse(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)));

        joystick.cross().onTrue(climber.runOnce(() -> climber.startMotorClimb()));
        joystick.cross().onFalse(climber.runOnce(() -> climber.stopMotor()));
        joystick.triangle().onTrue(climber.runOnce(() -> climber.startMotorLower()));
        joystick.triangle().onFalse(climber.runOnce(() -> climber.stopMotor()));

        algeaState.whileTrue(
            autoScoreFactory.getAlgeaRemoveCommand(VisionConstants.limelights[0])
            .finallyDo(() -> {
                M_ROBOT_STATE.setDriveState(DriveState.TELEOP);
                }) //REDENDUNCY TO ALWAYS SET BACK TO TELEOP AFTER REMOVAL
        );

        algeaState.onFalse(
            Commands.print("False").andThen(
            elevatorPivot.stowArm().alongWith(claw.fullHold()))
        );

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
                .andThen(drivetrain.driveBack())
                .andThen(elevatorPivot.stowArm().alongWith(claw.hold()))
        );

        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void registerPathPlannerCommands(){
        NamedCommands.registerCommand("autoScoreL4", 
            autoScoreFactory.AutonomousPeriodAutoScore(() -> 3, 
                VisionConstants.limelights[1], 
                VisionConstants.limelights[0]));
        
        NamedCommands.registerCommand("autoScoreL3", 
            autoScoreFactory.AutonomousPeriodAutoScore(() -> 2, 
                VisionConstants.limelights[1], 
                VisionConstants.limelights[0]));

        NamedCommands.registerCommand("autoScoreL2", 
            autoScoreFactory.AutonomousPeriodAutoScore(() -> 1, 
                VisionConstants.limelights[1], 
                VisionConstants.limelights[0]));
        
        NamedCommands.registerCommand("intake", 
            elevatorPivot.goToIntake()
                .andThen(claw.fullIntake()
                .andThen(Commands.waitUntil(claw.hasCoral))
                .andThen(claw.slowIntake())
                .andThen(Commands.waitUntil(claw.hasCoral.negate()))
                .andThen(claw.slowBackup())
                .andThen(Commands.waitUntil(claw.hasCoral))));
    }
}
