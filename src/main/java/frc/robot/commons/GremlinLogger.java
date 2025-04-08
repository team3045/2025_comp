// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commons;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class GremlinLogger extends DogLog {
    private static final String PID_KEY = "PID";

    // Create a NetworkTable instance and entry for the DEBUG flag
    private static final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private static final NetworkTable debugTable = ntInstance.getTable("Debug");
    private static final NetworkTableEntry debugEntry = debugTable.getEntry("Enabled");

    public static boolean DEBUG = false;
    private static Notifier debugNotifier;

    static {
        debugEntry.setBoolean(DEBUG);
        // Create a Notifier to update the DEBUG flag on a separate thread
        debugNotifier = new Notifier(() -> {
            DEBUG = debugEntry.getBoolean(false);
        });
        debugNotifier.startPeriodic(0.5); // Update every 500ms
    }

    public static void logTalonFX(String motorName, TalonFX motor) {
        GremlinLogger.debugLog(motorName + "/DeviceID", motor.getDeviceID());
        GremlinLogger.debugLog(motorName + "/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
        GremlinLogger.debugLog(motorName + "/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        GremlinLogger.debugLog(motorName + "/SupplyVoltage", motor.getSupplyVoltage().getValueAsDouble());
        GremlinLogger.debugLog(motorName + "/OutputVoltage", motor.getMotorVoltage().getValueAsDouble());
        GremlinLogger.debugLog(motorName + "/Positon", motor.getPosition().getValueAsDouble());
        GremlinLogger.debugLog(motorName + "/RotorPosition", motor.getRotorPosition().getValueAsDouble());
        GremlinLogger.debugLog(motorName + "/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        GremlinLogger.debugLog(motorName + "/Velocity", motor.getVelocity().getValueAsDouble());
        GremlinLogger.debugLog(motorName + "/Temperature", motor.getDeviceTemp().getValueAsDouble());
    }

    public static void logTalonFXPID(String motorName, TalonFX motor) {
        GremlinLogger.debugLog(motorName + PID_KEY + "/Error", motor.getClosedLoopError().getValueAsDouble());
        GremlinLogger.debugLog(motorName + PID_KEY + "/TotalOutput", motor.getClosedLoopOutput().getValueAsDouble());
        GremlinLogger.debugLog(motorName + PID_KEY + "/Feedforward", motor.getClosedLoopFeedForward().getValueAsDouble());
        GremlinLogger.debugLog(motorName + PID_KEY + "/POutput",
                motor.getClosedLoopProportionalOutput().getValueAsDouble());
        GremlinLogger.debugLog(motorName + PID_KEY + "/Ioutput", motor.getClosedLoopIntegratedOutput().getValueAsDouble());
        GremlinLogger.debugLog(motorName + PID_KEY + "/DOutput", motor.getClosedLoopDerivativeOutput().getValueAsDouble());
        GremlinLogger.debugLog(motorName + PID_KEY + "/Reference", motor.getClosedLoopReference().getValueAsDouble());
    }

    public static void debugLog(String key, double value) {
        if (DEBUG) {
            SmartDashboard.putNumber(key, value);
        }

        log(key,value);
    }

    public static void debugLog(String key, boolean value) {
        if (DEBUG) {
            SmartDashboard.putBoolean(key, value);
        }

        log(key,value);
    }

    public static void debugLog(String path, Vector<N3> stddevs) {
        if (DEBUG) {
            SmartDashboard.putNumber(path + "/Stddevs/XY", stddevs.getData()[0]);
            SmartDashboard.putNumber(path + "/Stddevs/Theta", stddevs.getData()[2]);
        }

        log(path + "/Stddevs/XY", stddevs.getData()[0]);
        log(path + "/Stddevs/Theta", stddevs.getData()[2]);
    }

    public static void debugLog(String path, Pose2d pose){
        if (DEBUG) {
            SmartDashboard.putNumberArray(path, new double[]{
                pose.getX(),
                pose.getY(),
                pose.getRotation().getRadians()
            });
        }

        log(path, pose);
    }


    public static boolean isDebug() {
        return DEBUG;
    }

    public static void updateDebug() {
        // Update the DEBUG value from NetworkTables
        DEBUG = debugEntry.getBoolean(false);
    }
}
