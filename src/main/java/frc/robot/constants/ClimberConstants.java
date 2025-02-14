package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConstants {
    public static final int climberID = 21;
    public static final String canbus = "Canivore 3045";

    public static final double climbDoneVelocityThreshold = 0.1;

    public static final double controlTimesyncFreq = 100;

    public static final int currentLimit = 40;

    public static final boolean climberStatorCurrentLimitEnable = true;
    public static final boolean climberSupplyCurrentLimitEnable = true;

    public static final double climberMaxAccelerationRotations = 2;
    public static final double climberMaxVelocityRotations = 5;

    public static final double climberClimbVelocity = 1;

    public static final MotorOutputConfigs MotorOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withControlTimesyncFreqHz(controlTimesyncFreq);
    
    public static final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(currentLimit)
        .withSupplyCurrentLimit(currentLimit)
        .withStatorCurrentLimitEnable(climberStatorCurrentLimitEnable)
        .withSupplyCurrentLimitEnable(climberSupplyCurrentLimitEnable);

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kA = 0;
    public static final double kV = 12 / 93.8;

    public static final MotionMagicConfigs climberMotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicAcceleration(climberMaxAccelerationRotations)
        .withMotionMagicCruiseVelocity(climberMaxVelocityRotations);

    public static final Slot0Configs climberSlot0Configs = new Slot0Configs()
        .withKA(kA)
        .withKD(kD)
        .withKG(kG)
        .withKI(kI)
        .withKP(kP)
        .withKS(kS)
        .withKV(kV);

    public static final FeedbackConfigs climberFeedbackConfigs = new FeedbackConfigs();

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        .withCurrentLimits(currentLimitConfigs)
        .withMotorOutput(MotorOutputConfigs)
        .withMotionMagic(climberMotionMagicConfigs)
        .withFeedback(climberFeedbackConfigs)
        .withSlot0(climberSlot0Configs);
}