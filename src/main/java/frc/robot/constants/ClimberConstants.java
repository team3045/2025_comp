package frc.robot.constants;

import static frc.robot.constants.ClawConstants.currentLimitConfigs;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConstants {
    public static final String canbus = "rio";
    public static final int climberID = 21;
    public static final float volts = 4;
    public static final float minAmps = 40;
    public static final float maxAmps = 80;

    public static final double climberSpeed = 60;

    public static final double speedTolerance = 0.5; // RPS

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
    public static final double proximityHysterisis = 0.01; // 1 cm
    public static final double proximityThreshold = 0.05; // 10cm

    public static final double outputWaitTime = 0.5; // 500 ms
    public static final int timeoutSeconds = 10; // 10 sec

    public static final double updateFrequency = 50; // every 20 ms, this is overridden to 100 Hz when we're in
                                                     // ShortRange100hz mode

    public static final double holdAlgeaVoltage = -0.8;

    public static final MotorOutputConfigs MotorOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withControlTimesyncFreqHz(controlTimesyncFreq);

    public static final CurrentLimitsConfigs cirrentLimitCOnfig = new CurrentLimitsConfigs()
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

    public static final double slot1kP = 10;
    public static final double slot1kI = 0;
    public static final double slot1kD = 0;
    public static final double slot1kG = 0;
    public static final double slot1kS = 0;
    public static final double slot1kA = 0;
    public static final double slot1kV = 12 / 93.8;

    public static final MotionMagicConfigs climberMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(clawMaxAccelerationRotations)
            .withMotionMagicCruiseVelocity(clawMaxVelocityRotations);

    public static final Slot0Configs climberSlot0Configs = new Slot0Configs()
            .withKA(kA)
            .withKD(kD)
            .withKG(kG)
            .withKI(kI)
            .withKP(kP)
            .withKS(kS)
            .withKV(kV);

    public static final Slot1Configs climberSlot1Configs = new Slot1Configs()
            .withKA(slot1kA)
            .withKD(slot1kD)
            .withKG(slot1kG)
            .withKI(slot1kI)
            .withKP(slot1kP)
            .withKS(slot1kS)
            .withKV(slot1kV);

    public static final FeedbackConfigs climberFeedbackConfigs = new FeedbackConfigs();

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withMotorOutput(MotorOutputConfigs)
            .withMotionMagic(climberMotionMagicConfigs)
            .withFeedback(climberFeedbackConfigs)
            .withSlot0(climberSlot0Configs)
            .withSlot1(climberSlot1Configs);

}


