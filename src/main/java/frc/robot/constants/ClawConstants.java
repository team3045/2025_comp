package frc.robot.constants;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;


public class ClawConstants {
    public static final int clawID = 15;
    public static final int canRangeId = 17;
    public static final int hopperId = 20;
    public static final String canbus = "Canivore 3045";


    public static final double intakeSpeed = 20;
    public static final double outtakeSpeed = 6;
    public static final double slowSpeed = 6;
    public static final double hopperSpeed = 20;
    public static final double algeaIntakeSpeed = -20;
    public static final double algeaOuttakeSpeed = 20;
    public static final double holdSpeed = 0;

    public static final double speedTolerance = 0.5; //RPS

    public static final double gearing = 30 / 12;

    public static final double flywheelMOI = 0.00024086;
    public static final double controlTimesyncFreq = 100;

    public static final int currentLimit = 40; 

    public static final boolean clawStatorCurrentLimitEnable = true;
    public static final boolean clawSupplyCurrentLimitEnable = true;

    public static final double clawMaxAccelerationRotations = 2;
    public static final double clawMaxVelocityRotations = 5;

    public static final double fovCenterX = 0;
    public static final double fovCenterY = 0;
    public static final double fovRangeX = 6.75;
    public static final double fovRangeY = 6.75;

    public static final double minSignalStrength = 0;
    public static final double proximityHysterisis = 0.01; //1 cm
    public static final double proximityThreshold = 0.1; //10cm

    public static final double outputWaitTime = 0.5; //500 ms
    public static final int timeoutSeconds = 10; //10 sec

    public static final double updateFrequency = 50; //every 20 ms, this is overridden to 100 Hz when we're in ShortRange100hz mode

    public static final double holdAlgeaVoltage = -0.8;

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
    

    public static final MotorOutputConfigs MotorOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withControlTimesyncFreqHz(controlTimesyncFreq);
    
    public static final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(currentLimit)
        .withSupplyCurrentLimit(currentLimit)
        .withStatorCurrentLimitEnable(clawStatorCurrentLimitEnable)
        .withSupplyCurrentLimitEnable(clawSupplyCurrentLimitEnable);

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kA = 0;
    public static final double kV = 12 / 93.8;

    public static final MotionMagicConfigs clawMotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicAcceleration(clawMaxAccelerationRotations)
        .withMotionMagicCruiseVelocity(clawMaxVelocityRotations);

    public static final Slot0Configs clawSlot0Configs = new Slot0Configs()
        .withKA(kA)
        .withKD(kD)
        .withKG(kG)
        .withKI(kI)
        .withKP(kP)
        .withKS(kS)
        .withKV(kV);

    public static final FeedbackConfigs clawFeedbackConfigs = new FeedbackConfigs();

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        .withCurrentLimits(currentLimitConfigs)
        .withMotorOutput(MotorOutputConfigs)
        .withMotionMagic(clawMotionMagicConfigs)
        .withFeedback(clawFeedbackConfigs)
        .withSlot0(clawSlot0Configs);


     
    
}
