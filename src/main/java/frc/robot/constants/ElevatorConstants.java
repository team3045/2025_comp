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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
    public static final int rightMotorId = 15;
    public static final int leftMotorId = 16;
    public static final String canbus = "Canivore 3045";

    public static final double numStages = 4;
    public static final double firstStageLength = Units.inchesToMeters(24); //m
    public static final double secondStageLength = Units.inchesToMeters(24); //m
    public static final double thirdStageLength = Units.inchesToMeters(24); //m
    public static final double fourthStageLength = Units.inchesToMeters(24); //m

    //Cancoder is 1:1 with drum so rotorToSensor is equivalent to the total gear ratio
    public static final double rotorToSensorRatio = 1; 
    public static final double sensorToMechanismRatio = (686.0 / 27.0);
    public static final double totalGearing = rotorToSensorRatio * sensorToMechanismRatio;

    public static final double minimumHeight = fourthStageLength; //m

    public static final double maxHeight = firstStageLength + secondStageLength + thirdStageLength + fourthStageLength; // m

    public static final double heightTolerance = 0.03; //3cm

    public static final double statorCurrentLimit = 60; //Amps
    public static final double supplyCurrentLimit = 80; //Amps
    public static final double supplyCurrentLimitLowerLimit = 60; //Amps
    public static final double supplyCurrentLimitLowerLimitTime = 1; //second
    public static final boolean statorCurrentLimitEnable = true;
    public static final boolean supplyCurrentLimitEnable = true;

    /*Simulation */
    public static final double carriageMass = 4.53592 * 2; //kg, 9kg = 20lbs
    public static final double drumRadius = .2; //m
    public static final double canvasWidth = 2; //m
    public static final double canvasHeight = 6; //m


    //Rotation of the output shaft. To get rotations of motor to the height of elevator we need to multiply by the gear ratio
    public static final double rotationToLengthRatio = (2 * Math.PI * drumRadius) / 1; //1.2566370614359172m / 1 rot

    public static final double maxAccelerationLinear = 2; //m per sec^2
    public static final double maxVelocityLinear = 1; //m per sec
    public static final double maxAccelerationRotations =  maxAccelerationLinear / rotationToLengthRatio; //rot per sec^2
    public static final double maxVelocityRotations = maxVelocityLinear / rotationToLengthRatio; //rot per sec

    public static final double timesyncFrequency = 200; //Hz aka every 5 ms

    public static final InvertedValue leftInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue rightInverted = InvertedValue.CounterClockwise_Positive;

    public static final double kP = 200;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kA = 0;
    public static final double kV = 0;

    /*Configuration */
    public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(statorCurrentLimit)
        .withSupplyCurrentLimit(supplyCurrentLimit)
        .withStatorCurrentLimitEnable(statorCurrentLimitEnable)
        .withSupplyCurrentLimitEnable(supplyCurrentLimitEnable)
        .withSupplyCurrentLowerTime(supplyCurrentLimitLowerLimitTime)
        .withSupplyCurrentLowerLimit(supplyCurrentLimitLowerLimit);

    public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withRotorToSensorRatio(rotorToSensorRatio)
        .withSensorToMechanismRatio(sensorToMechanismRatio);

    public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicAcceleration(maxAccelerationRotations)
        .withMotionMagicCruiseVelocity(maxVelocityRotations); //Consider adding jerk or making it expo

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
