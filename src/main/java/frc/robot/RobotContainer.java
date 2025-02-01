// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState.DriveState;
import frc.robot.commands.AutoScoreFactory;
import frc.robot.commands.DriveWheelRadiusCharacterization;
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
    private final Trigger disableGlobalEstimation = scoringState.and(() -> drivetrain.withinDistanceOfReef(FieldConstants.reefDistanceTolerance));


    public RobotContainer() {
        DogLog.setOptions(new DogLogOptions());
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

        joystick.circle().onTrue(elevatorPivot.goToIntakeReady());
        joystick.triangle().onTrue(intakeSequenceFactory.moveElevatorAndIntake());

        joystick.cross().whileTrue(new DriveWheelRadiusCharacterization(drivetrain, DriveWheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE));

        joystick.square().onTrue(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.AUTOSCORE)));
        joystick.square().onFalse(Commands.runOnce(() -> M_ROBOT_STATE.setDriveState(DriveState.TELEOP)));
        
        scoringState.whileTrue(
            autoScoreFactory.pathFindWithApriltagFeeback(VisionConstants.limelights[0], VisionConstants.limelights[1]) //righ and left
            .alongWith(autoScoreFactory.setElevatorHeight())
            .andThen(elevatorPivot.goDownToScore())
            .andThen(Commands.waitSeconds(0.3))
            .andThen(claw.clawOutake())
            .andThen(Commands.waitSeconds(0.2))
            .andThen(drivetrain.driveBack())
            .finallyDo(() -> {
                M_ROBOT_STATE.setDriveState(DriveState.TELEOP);
                }) //REDENDUNCY TO ALWAYS SET BACK TO TELEOP AFTER SCORE
            );

        scoringState.onFalse(
            elevatorPivot.stowArm().alongWith(claw.stop())); //STOW ARM AND STOP CLAW AFTER SCORING
        
        disableGlobalEstimation.onTrue(Commands.runOnce(() -> vision.setRejectAllUpdates(true)));
        disableGlobalEstimation.onFalse(Commands.runOnce(() -> vision.setRejectAllUpdates(false)));
                
        // joystick.circle().whileTrue(elevatorPivot.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // joystick.cross().whileTrue(elevatorPivot.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // joystick.square().whileTrue(elevatorPivot.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // joystick.triangle().whileTrue(elevatorPivot.sysIdDynamic(SysIdRoutine.Direction.kReverse));


        //joystick.share().onTrue(elevatorPivot.goToIntakeReady());
        joystick.L1().onTrue(autoScoreFactory.setElevatorHeight());
        joystick.R1().onTrue(elevatorPivot.stowArm().alongWith(claw.stop()));
        joystick.R2().onTrue(elevatorPivot.goToPosition(() -> elevatorPivot.getHeight() - 0.15, () -> elevatorPivot.getPivotAngleDegrees())
        .andThen(Commands.waitSeconds(0.3).andThen(claw.clawOutake().andThen(Commands.waitSeconds(0.2)))
        .andThen(drivetrain.driveBack().andThen(elevatorPivot.stowArm().alongWith(claw.stop())))));
        // joystick.square().onTrue(elevatorPivot.zeroHeight());
        //joystick.L2().whileTrue(elevatorPivot.decreaseAngle().repeatedly());
        // joystick.R2().whileTrue(elevatorPivot.increaseAngle().repeatedly());

        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Curvy Square");
    }
}
