package org.tahomarobotics.robot.climber;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConstants {
    public static final double MOTOR_TO_CLIMBER_GEAR_RATIO = (8.0 / 72.0) * (20.0 / 72.0) * (16.0 / 48.0);
    public static final double CLIMBER_ZERO_POSITION = 0;

    public static final double STOW_POSITION = 0.04;
    public static final double DEPLOY_POSITION = 0.16;
    public static final double CLIMB_POSITION = -0.02;

    public static final double CLIMB_POSITION_TOLERANCE = 0.005;

    public static final double RATCHET_SOLENOID_DEPLOY_TIME = 0.5;
    public static final double RATCHET_SOLENOID_DEPLOY_PERCENTAGE = 1;

    private static final double CLIMBER_MAX_VELOCITY = 0.125;
    private static final double CLIMBER_MAX_ACCELERATION = CLIMBER_MAX_VELOCITY * 4;
    private static final double CLIMBER_MAX_JERK = CLIMBER_MAX_ACCELERATION * 4;

    public static final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs() // Unladen
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(29.236)
                .withKD(1.4369)
                .withKS(0.11051)
                .withKV(10.452)
                .withKA(0.14674)
        )
        .withSlot1(
            new Slot1Configs() // Laden
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(29.236 * 12)
                .withKI(75)
                .withKD(1.4369)
                .withKS(0.11051)
                .withKV(10.452)
                .withKA(0.14674)
        )
        .withMotorOutput(new MotorOutputConfigs()
                             .withNeutralMode(NeutralModeValue.Coast)
                             .withInverted(InvertedValue.Clockwise_Positive))
        .withMotionMagic(new MotionMagicConfigs()
                             .withMotionMagicCruiseVelocity(CLIMBER_MAX_VELOCITY)
                             .withMotionMagicAcceleration(CLIMBER_MAX_ACCELERATION)
                             .withMotionMagicJerk(CLIMBER_MAX_JERK))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / MOTOR_TO_CLIMBER_GEAR_RATIO))
        .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}