// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorPivotConstants {
    public static final int rightMotorId = 13;
    public static final int leftMotorId = 12;
    public static final int pivotMotorId = 14;
    public static final int pivotCancoderId = 16;
    public static final int canRangeId = 18;
    public static final String canbus = "Canivore 3045";

    public static final String elevatorTable = "elevator";
    public static final String pivotTable = "pivot";

    public static final double numStages = 4;
    public static final double firstStageLength = Units.inchesToMeters(24); //m
    public static final double secondStageLength = Units.inchesToMeters(24); //m
    public static final double thirdStageLength = Units.inchesToMeters(24); //m
    public static final double fourthStageLength = Units.inchesToMeters(24); //m

    public static final double pivotArmLength = Units.inchesToMeters(12.632); //m
    public static final double carriageHeightToPivot = Units.inchesToMeters(-1.25);
    public static final double minAngleDegrees = -73;
    public static final double maxAngleDegrees = 120;
    public static final double stowAngle = 119;
    public static final double intakingAngle = 119;
    public static final double processingAngle = -65;

    /*Collision */
    public static final double maxUpperCollisionAngle = 84;
    public static final double maxAlgeaCollisionAngle = 20;
    public static final double algeaTravelAngle = 10;
    public static final double travelAngle = maxUpperCollisionAngle - 10;
    public static final double stageToCarriageMax = 0.15;

    
    public static final double rotorToSensorRatio = 1.0; 
    public static final double sensorToMechanismRatio = (56.0 / 12.0);
    public static final double totalGearing = rotorToSensorRatio * sensorToMechanismRatio;

    public static final double pivotRotorToSensorRatio = (56 * 56 * 48) / (12.0 * 18 * 24); 
    public static final double pivotSensorToMechanismRatio = 1;
    public static final double pivotTotalGearing = pivotSensorToMechanismRatio * pivotRotorToSensorRatio;

    public static final double carriageToGround = 0.476306;//Units.inchesToMeters(16.752); //This is from the top of carriage to the ground, when at lowest position
    public static final double minimumHeight = carriageToGround; //m
    public static final double stowHeight = minimumHeight;
    public static final double intakingReadyHeight = 0.973; 
    public static final double intakingHeight = minimumHeight;
    public static final double processingHeight = minimumHeight+0.1;

    public static final double maxHeight = 2.100; // m

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
    public static final double carriageMass = 17.65;
    public static final double drumRadius = 0.02864739; //m
    public static final double canvasWidth = 2; //m
    public static final double canvasHeight = 6; //m
    public static final double pivotMOI = 0.08585723; //moment of inertia Kg * m^2

    public static final double stage3StageLength = Units.inchesToMeters(23);
    public static final double stage2StageLength = Units.inchesToMeters(21);
    public static final double verticalTimerThreshold = 0.5;

    public static final double pivotOffsetX = 0;
    public static final double pivotOffsetY = Units.inchesToMeters(-11);
    public static final double pivotOffsetZ = Units.inchesToMeters(17.5);


    //Rotation of the output shaft. To get rotations of motor to the height of elevator we need to multiply by the gear ratio
    public static final double rotationToLengthRatio = (2 * Math.PI * drumRadius) / 1; //1.2566370614359172m / 1 rot //0.05729478

    public static final double maxAccelerationLinear = 4; //m per sec^2
    public static final double maxVelocityLinear = 2; //m per sec
    public static final double maxAccelerationRotations =  maxAccelerationLinear / rotationToLengthRatio; //rot per sec^2
    public static final double maxVelocityRotations = maxVelocityLinear / rotationToLengthRatio; //rot per sec

    public static final double pivotMaxAcceleration =  2; //rot per sec^2
    public static final double pivotMaxVelocity = 2; //rot per sec

    public static final double timesyncFrequency = 200; //Hz aka every 5 ms

    public static final InvertedValue leftInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;

    public static final double kP = 8;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0.3;
    public static final double kS = 0.1;
    public static final double kA = 0;
    public static final double kV = 0.60932;

    public static final double pivotKP = 45;
    public static final double pivotKI = 0;
    public static final double pivotKD = 0.3;
    public static final double pivotKG = 0.4;
    public static final double pivotKS = 0.08;
    public static final double pivotKA = 0;
    public static final double pivotKV = 2.8;

    public static final double magnetOffset = -0.193;
    public static final SensorDirectionValue pivotEncoderSensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    public enum HeightPositions{
        L4(1.985),
        L3(1.187),
        L2(0.82),
        LOW_ALGEA(1.032),
        HIGH_ALGEA(1.487);

        private final double height;
        HeightPositions(double height){
            this.height = height;
        }

        public double getHeight(){
            return height;
        }
    }

    public enum AnglePositions{
        L4(46),
        L3(83),
        L2(87),
        LOW_ALGEA(-70),
        HIGH_ALGEA(-70);

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
        .withRemoteCANcoder(new CoreCANcoder(pivotCancoderId, canbus))
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


    public static final double fovCenterX = 0;
    public static final double fovCenterY = 0;
    public static final double fovRangeX = 6.75;
    public static final double fovRangeY = 6.75;

    public static final double minSignalStrength = 0;
    public static final double proximityHysterisis = 0.01; //1 cm
    public static final double proximityThreshold = 0.075; //7.5cm

    public static final double updateFrequency = 50; //every 20 ms, this is overridden to 100 Hz when we're in ShortRange100hz mode
    public static final double debounceTime = 0.4;

    public static final FovParamsConfigs fovConfigs = new FovParamsConfigs()
        .withFOVCenterX(fovCenterX)
        .withFOVCenterY(fovCenterY)
        .withFOVRangeX(fovRangeX)
        .withFOVRangeY(fovRangeY);
    
    public static final ProximityParamsConfigs proximityConfigs = new ProximityParamsConfigs()
            .withMinSignalStrengthForValidMeasurement(minSignalStrength)
            .withProximityHysteresis(proximityHysterisis)
            .withProximityThreshold(proximityThreshold);
    
    public static final ToFParamsConfigs tofConfigs = new ToFParamsConfigs()
        .withUpdateFrequency(updateFrequency)
        .withUpdateMode(UpdateModeValue.ShortRange100Hz);
    
    public static final CANrangeConfiguration canRangeConfig = new CANrangeConfiguration()
        .withFovParams(fovConfigs)
        .withProximityParams(proximityConfigs)
        .withToFParams(tofConfigs);
}