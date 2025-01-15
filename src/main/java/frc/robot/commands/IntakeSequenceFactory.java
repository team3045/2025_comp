package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.IntakeSequenceConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;

public class IntakeSequenceFactory {
    private CommandSwerveDrivetrain drivetrain; 
    private ElevatorPivot elevatorPivot;
    private Claw claw; 

    private Boolean pastMidPoint;


    public IntakeSequenceFactory(CommandSwerveDrivetrain drivetrain, ElevatorPivot elevatorPivot, Claw claw){
        this.drivetrain = drivetrain;
        this.elevatorPivot = elevatorPivot;
        this.claw = claw; 

        pastMidPoint = drivetrain.getState().Pose.getX() < 5; //TODO: I dont think this will work because will always use state at construction
    }

    public Command getPathFindCommand(){
        return new ConditionalCommand(
            drivetrain.pathFindToPose(
               IntakeSequenceConstants.leftSubstationPose, 
                IntakeSequenceConstants.desiredEndVelocity), //TODO: personally I would prefer these be constant doubles and Pose2ds and you put the lambda here
            drivetrain.pathFindToPose(
                IntakeSequenceConstants.rightSubstationPose,
                IntakeSequenceConstants.desiredEndVelocity),
            () -> pastMidPoint); //TODO: I don't think this condition makes sense
    } 

    public Command setElevatorPivotPosition(){
        return new SequentialCommandGroup( //TODO: use and then sugaring, also "goTo" Methods already wait until its at the height before ending
            elevatorPivot.goToHeight(IntakeSequenceConstants.intakeReadyHeight) //TODO: Use GoToPosition rather than goToHeight or GoToAngle, 
            .andThen(Commands.waitUntil(elevatorPivot.atTargetHeight)),
            elevatorPivot.goToAngleDegrees(IntakeSequenceConstants.intakeReadyAngle)) //TODO: Use GoToPosition rather than goToHeight or GoToAngle, 
            .andThen(Commands.waitUntil(elevatorPivot.atTargetAngle)
        );
    }

    public Command moveElevatorAndIntake(){
        return 
            elevatorPivot.goToHeight(IntakeSequenceConstants.intakingHeight) //TODO: Use GoToPosition rather than goToHeight or GoToAngle, 
            .alongWith(claw.clawIntake())
            .andThen(Commands.waitUntil(claw.hasObject))
                .withTimeout(IntakeSequenceConstants.timeOutTime) //TODO: only do this if sim
            .andThen(elevatorPivot.goToPosition
                (IntakeSequenceConstants.stowHeight, //TODO: Make this a unique command called like goToStow or something
                IntakeSequenceConstants.stowAngle));
    }

}
