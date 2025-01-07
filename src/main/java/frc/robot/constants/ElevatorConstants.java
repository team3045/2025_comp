// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
    public static final int rightMotorId = 0;
    public static final int leftMotorId = 1;
    public static final int cancoderId = 2;
    public static final String canbus = "Canivore 3045";

    public static final double numStages = 4;
    public static final double firstStageLength = Units.inchesToMeters(24); //m
    public static final double secondStageLength = Units.inchesToMeters(24); //m
    public static final double thirdStageLength = Units.inchesToMeters(24); //m
    public static final double fourthStageLength = Units.inchesToMeters(24); //m

    public static final double rotationToLengthRatio = 0.1; //1 rotation = 0.1m
    public static final double minimumHeight = 0.15; //m

    public static final double maxHeight = 1.5; // m

    public static final double heightTolerance = 0.03; //3cm
}
