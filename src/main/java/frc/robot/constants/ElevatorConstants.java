// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

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

    public static final double rotorToSensorRatio = 1;
    public static final double sensorToMechanismRatio = 1;

    public static final double rotationToLengthRatio = 0.1; //1 rotation = 0.1m
    public static final double minimumHeight = 0.15; //m

    public static final double maxHeight = 1.5; // m

    public static final double heightTolerance = 0.03; //3cm

    public static final double statorCurrentLimit = 60; //Amps
    public static final double supplyCurrentLimit = 80; //Amps
    public static final double supplyCurrentLimitLowerLimit = 60; //Amps
    public static final double supplyCurrentLimitLowerLimitTime = 1; //second
    public static final boolean statorCurrentLimitEnable = true;
    public static final boolean supplyCurrentLimitEnable = true;

    public static final double maxAccelerationLinear = 1; //m per sec^2
    public static final double maxVelocityLinear = 1; //m per sec
    public static final double maxAccelerationRotations =  maxAccelerationLinear / rotationToLengthRatio; //rot per sec^2
    public static final double maxVelocityRotations = maxVelocityLinear / rotationToLengthRatio; //rot per sec

    public static final double timesyncFrequency = 200; //Hz aka every 5 ms

    public static final InvertedValue leftInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightInverted = InvertedValue.CounterClockwise_Positive;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kA = 0;
    public static final double kV = 0;


    public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(statorCurrentLimit)
        .withSupplyCurrentLimit(supplyCurrentLimit)
        .withStatorCurrentLimitEnable(statorCurrentLimitEnable)
        .withSupplyCurrentLimitEnable(supplyCurrentLimitEnable)
        .withSupplyCurrentLowerTime(supplyCurrentLimitLowerLimitTime)
        .withSupplyCurrentLowerLimit(supplyCurrentLimitLowerLimit);

    public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withFusedCANcoder(new CoreCANcoder(cancoderId, canbus))
        .withRotorToSensorRatio(rotorToSensorRatio)
        .withSensorToMechanismRatio(sensorToMechanismRatio);

    public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicAcceleration(maxAccelerationRotations)
        .withMotionMagicCruiseVelocity(maxVelocityRotations)
        .withMotionMagicExpo_kA(cancoderId);

    public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
        .withControlTimesyncFreqHz(timesyncFrequency)
        .withNeutralMode(NeutralModeValue.Brake);

    public static final Slot0Configs slot0Configs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKA(kA)
        .withKD(kD)
        .withKG(kG)
        .withKI(kI)
        .withKP(kP)
        .withKS(kS)
        .withKV(kV)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    
    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        .withCurrentLimits(currentLimits)
        .withFeedback(feedbackConfigs)
        .withMotionMagic(motionMagicConfigs)
        .withMotorOutput(motorOutputConfigs)
        .withSlot0(slot0Configs);
}
