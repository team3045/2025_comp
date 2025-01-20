// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorPivotConstants {
    public static final int rightMotorId = 15;
    public static final int leftMotorId = 16;
    public static final int pivorMotorId = 17;
    public static final int pivotCancoderId = 18;
    public static final String canbus = "Canivore 3045";

    public static final String elevatorTable = "elevator";
    public static final String pivotTable = "pivot";

    public static final double numStages = 4;
    public static final double firstStageLength = Units.inchesToMeters(24); //m
    public static final double secondStageLength = Units.inchesToMeters(24); //m
    public static final double thirdStageLength = Units.inchesToMeters(24); //m
    public static final double fourthStageLength = Units.inchesToMeters(24); //m

    public static final double pivotArmLength = Units.inchesToMeters(20.125); //m
    public static final double carriageHeightToPivot = Units.inchesToMeters(-1.25);
    public static final double minAngleDegrees = -90;
    public static final double maxAngleDegrees = 90;
    public static final double stowAngle = maxAngleDegrees;
    public static final double intakingAngle = -90;

    
    public static final double rotorToSensorRatio = 1; 
    public static final double sensorToMechanismRatio = (56.0 / 12.0);
    public static final double totalGearing = rotorToSensorRatio * sensorToMechanismRatio;

    public static final double pivotRotorToSensorRatio = 50; 
    public static final double pivotSensorToMechanismRatio = 1;
    public static final double pivotTotalGearing = pivotSensorToMechanismRatio * pivotRotorToSensorRatio;

    public static final double carriageToGround = 0.508;//Units.inchesToMeters(16.752); //This is from the top of carriage to the ground, when at lowest position
    public static final double minimumHeight = carriageToGround; //m
    public static final double stowHeight = minimumHeight;
    public static final double intakingReadyHeight = 0.85; 
    public static final double intakingHeight = 0.75;

    public static final double maxHeight = Units.inchesToMeters(57) + carriageToGround; // m

    public static final double heightTolerance = 0.03; //3cm
    public static final double angleToleranceDegrees = 0.5; //0.5 degree tolerance

    public static final double statorCurrentLimit = 60; //Amps
    public static final double supplyCurrentLimit = 80; //Amps
    public static final double supplyCurrentLimitLowerLimit = 60; //Amps
    public static final double supplyCurrentLimitLowerLimitTime = 1; //second
    public static final boolean statorCurrentLimitEnable = true;
    public static final boolean supplyCurrentLimitEnable = true;

    public static final double pivotStatorCurrentLimit = 60; //Amps
    public static final double pivotSupplyCurrentLimit = 80; //Amps
    public static final double pivotSupplyCurrentLimitLowerLimit = 60; //Amps
    public static final double pivotSupplyCurrentLimitLowerLimitTime = 1; //second
    public static final boolean pivotStatorCurrentLimitEnable = true;
    public static final boolean pivotSupplyCurrentLimitEnable = true;


    /*Simulation */
    public static final double carriageMass = 4.53592 * 2; //kg, 9kg = 20lbs
    public static final double drumRadius = 0.02794; //m
    public static final double canvasWidth = 2; //m
    public static final double canvasHeight = 6; //m
    public static final double pivotMOI = 0.02347363; //moment of inertia Kg * m^2

    public static final double stage3StageLength = Units.inchesToMeters(19);
    public static final double stage2StageLength = stage3StageLength + Units.inchesToMeters(20);
    public static final double verticalTimerThreshold = 0.5;

    public static final double pivotOffsetX = Units.inchesToMeters(8.250);
    public static final double pivotOffsetY = 0;
    public static final double pivotOffsetZ = Units.inchesToMeters(15.502224);


    //Rotation of the output shaft. To get rotations of motor to the height of elevator we need to multiply by the gear ratio
    public static final double rotationToLengthRatio = (2 * Math.PI * drumRadius) / 1; //1.2566370614359172m / 1 rot //0.05729478

    public static final double maxAccelerationLinear = 2; //m per sec^2
    public static final double maxVelocityLinear = 1; //m per sec
    public static final double maxAccelerationRotations =  maxAccelerationLinear / rotationToLengthRatio; //rot per sec^2
    public static final double maxVelocityRotations = maxVelocityLinear / rotationToLengthRatio; //rot per sec

    public static final double pivotMaxAcceleration =  2; //rot per sec^2
    public static final double pivotMaxVelocity = 1; //rot per sec

    public static final double timesyncFrequency = 200; //Hz aka every 5 ms

    public static final InvertedValue leftInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue rightInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;

    public static final double kP = 5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0.38967;
    public static final double kS = 0.096618;;
    public static final double kA = 0.024757;
    public static final double kV = 0.60932;

    public static final double pivotKP = 30;
    public static final double pivotKI = 0;
    public static final double pivotKD = 0;
    public static final double pivotKG = 0.4;
    public static final double pivotKS = 0.1;
    public static final double pivotKA = 0.2;
    public static final double pivotKV = 1.5;

    public static final double magnetOffset = 0.371338;
    public static final SensorDirectionValue pivotEncoderSensorDirection = SensorDirectionValue.Clockwise_Positive;

    public enum HeightPositions{
        L4(1.73),
        L3(1.155),
        L2(0.694);

        private final double height;
        HeightPositions(double height){
            this.height = height;
        }

        public double getHeight(){
            return height;
        }
    }

    public enum AnglePositions{
        L4(35),
        L3(37),
        L2(37);

        private final double angle;
        AnglePositions(double angle){
            this.angle = angle;
        }

        public double getAngle(){
            return angle;
        }
    }

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
    
    public static final TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(currentLimits)
        .withFeedback(feedbackConfigs)
        .withMotionMagic(motionMagicConfigs)
        .withMotorOutput(motorOutputConfigs)
        .withSlot0(slot0Configs);

    public static final CurrentLimitsConfigs pivotCurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(pivotStatorCurrentLimit)
        .withSupplyCurrentLimit(pivotSupplyCurrentLimit)
        .withStatorCurrentLimitEnable(pivotStatorCurrentLimitEnable)
        .withSupplyCurrentLimitEnable(pivotSupplyCurrentLimitEnable)
        .withSupplyCurrentLowerTime(pivotSupplyCurrentLimitLowerLimitTime)
        .withSupplyCurrentLowerLimit(pivotSupplyCurrentLimitLowerLimit);
    
    public static final FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs()
        .withFusedCANcoder(new CoreCANcoder(pivotCancoderId, canbus))
        .withRotorToSensorRatio(pivotRotorToSensorRatio)
        .withSensorToMechanismRatio(pivotSensorToMechanismRatio);

    public static final MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicAcceleration(pivotMaxAcceleration)
        .withMotionMagicCruiseVelocity(pivotMaxVelocity); //Consider adding jerk or making it expo

    public static final MotorOutputConfigs pivotMotorOutputConfigs = new MotorOutputConfigs()
        .withControlTimesyncFreqHz(timesyncFrequency)
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(pivotInvert);

    public static final Slot0Configs pivotSlot0Configs = new Slot0Configs()
        .withKA(pivotKA)
        .withKD(pivotKD)
        .withKG(pivotKG)
        .withKI(pivotKI)
        .withKP(pivotKP)
        .withKS(pivotKS)
        .withKV(pivotKV)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withGravityType(GravityTypeValue.Arm_Cosine);
    
    public static final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration()
        .withCurrentLimits(pivotCurrentLimits)
        .withFeedback(pivotFeedbackConfigs)
        .withMotionMagic(pivotMotionMagicConfigs)
        .withMotorOutput(pivotMotorOutputConfigs)
        .withSlot0(pivotSlot0Configs);

    public static final CANcoderConfiguration pivotCancoderConfig = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withSensorDirection(pivotEncoderSensorDirection)
            .withAbsoluteSensorDiscontinuityPoint(0.5) //[-0.5,0.5]
            .withMagnetOffset(magnetOffset)
        );
}