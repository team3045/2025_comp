// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class GremlinRobotState {

    private DriveState driveState;
    private static GremlinRobotState mRobotState;

    public enum DriveState{
        TELEOP,
        AUTOSCORE,
        INTAKE,
        ALGEA
    }

    private GremlinRobotState(){
        this.driveState = DriveState.TELEOP;
    }

    public static GremlinRobotState getRobotState(){
        if(mRobotState == null){
            mRobotState = new GremlinRobotState();
        }

        return mRobotState;
    }

    public DriveState getDriveState(){
        return driveState;
    }

    public void setDriveState(DriveState newDriveState){
        this.driveState = newDriveState;
        SmartDashboard.putBoolean("Algea State", getDriveState() == DriveState.ALGEA);
        SmartDashboard.putBoolean("Intake State", getDriveState() == DriveState.INTAKE);
        SmartDashboard.putBoolean("Teleop State", getDriveState() == DriveState.TELEOP);
        SmartDashboard.putBoolean("Scoring State", getDriveState() == DriveState.AUTOSCORE);
    }
}
