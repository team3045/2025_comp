package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DynamicPathfindCommand extends Command {
    private final Supplier<Pose2d> targetPoseSupplier;
    private final DoubleSupplier desiredEndVelocitySupplier;
    private final PathConstraints constraints;
    private Command currentPathfindCommand;

    public DynamicPathfindCommand(
            Supplier<Pose2d> targetPoseSupplier,
            DoubleSupplier desiredEndVelocitySupplier,
            PathConstraints constraints) {
        this.targetPoseSupplier = targetPoseSupplier;
        this.desiredEndVelocitySupplier = desiredEndVelocitySupplier;
        this.constraints = constraints;
    }

    @Override
    public void initialize() {
        updatePathfindCommand();
        if (currentPathfindCommand != null) {
            currentPathfindCommand.initialize();
        }
    }

    @Override
    public void execute() {
        if (currentPathfindCommand != null) {
            currentPathfindCommand.execute();
        }
        System.out.println("DynamicPathfindCommand execute");
    }

    @Override
    public void end(boolean interrupted) {
        if (currentPathfindCommand != null) {
            currentPathfindCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return currentPathfindCommand != null && currentPathfindCommand.isFinished();
    }

    private void updatePathfindCommand() {
        // Get the dynamic pose and velocity
        Pose2d targetPose = targetPoseSupplier.get();
        double desiredEndVelocity = desiredEndVelocitySupplier.getAsDouble();

        // Create a new pathfinding command with the updated values
        currentPathfindCommand = AutoBuilder.pathfindToPose(targetPose, constraints, desiredEndVelocity);
    }
}
