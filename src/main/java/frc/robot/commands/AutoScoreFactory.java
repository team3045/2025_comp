// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.constants.DriveConstants.drive;

import java.util.function.Supplier;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorPivot;
import frc.robot.constants.AutoScoreConstants;
import frc.robot.subsystems.Claw;

/** Add your docs here. */
public class AutoScoreFactory {
    private static CommandSwerveDrivetrain drivetrain;
    private static ElevatorPivot elevatorPivot;
    private static Claw claw;
    private static Supplier<Integer> poleNumSub;
    private static Supplier<Integer> heightSub;

    public AutoScoreFactory(CommandSwerveDrivetrain Drivetrain, ElevatorPivot ElevatorPivot, Claw Claw, Supplier<Integer> PoleNumSub, Supplier<Integer> HeightSub) {
        drivetrain = Drivetrain;
        elevatorPivot = ElevatorPivot;
        claw = Claw;
        poleNumSub = PoleNumSub;
        heightSub = HeightSub;
    }

    public Command driveToScorePose() {
        return new DriveToPose(drivetrain, () -> drivetrain.getState().Pose, () -> AutoScoreConstants.kScorePoseMap.get(poleNumSub.get()));
    }

    public Command elevatorPivotGoToPose() {
        return elevatorPivot.goToPosition(() -> AutoScoreConstants.kScoreHeightMap.get(heightSub.get()), () -> AutoScoreConstants.kScoreAngleMap.get(heightSub.get()));
    }
    
    public Command ejectCoral() {
        return claw.clawOutake();
    }

    public Command stopClaw() {
        return claw.stop();
    }

    public Command stow() {
        return elevatorPivot.stowArm();
    }

    public Command autoScore() {
        return driveToScorePose().alongWith(elevatorPivotGoToPose()).andThen().andThen(ejectCoral()).andThen(drivetrain.driveBack()).andThen(stow()).andThen(stopClaw());
    }
}
