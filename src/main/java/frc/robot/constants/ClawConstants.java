package frc.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ClawConstants {
    public static final int clawID = 0;
    public static final String canbus = "Canivore 3045";

    public static final double intakeSpeed = 20;
    public static final double outtakeSpeed = -10;
    public static final double holdSpeed = 0;

    public static final double gearing = 30 / 12;

    public static final double flywheelMOI = 0.00024086;
    public static final double controlTimesyncFreq = 100;

    public static final int currentLimit = 40; 

    public static final boolean clawStatorCurrentLimitEnable = true;
    public static final boolean clawSupplyCurrentLimitEnable = true;

    public static final double clawMaxAccelerationRotations = 2;
    public static final double clawMaxVelocityRotations = 5;

    

    public static final MotorOutputConfigs MotorOutputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withControlTimesyncFreqHz(controlTimesyncFreq);
    
    public static final CurrentLimitsConfigs currentLimitConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(currentLimit)
        .withSupplyCurrentLimit(currentLimit)
        .withStatorCurrentLimitEnable(clawStatorCurrentLimitEnable)
        .withSupplyCurrentLimitEnable(clawSupplyCurrentLimitEnable);

    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kA = 0.06;
    public static final double kV = 12 / 100.5;

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
