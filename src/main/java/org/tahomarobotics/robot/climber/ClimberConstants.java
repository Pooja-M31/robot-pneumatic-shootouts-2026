/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.climber;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConstants {
    // TODO: 1 / 78; DONT FIX
    public static final double MOTOR_TO_CLIMBER_GEAR_RATIO = (8.0 / 72.0) * (18.0 / 72.0) * (16.0 / 48.0);

    public static final double ZERO_POSITION = 0.0;
    public static final double STOW_POSITION = 0.190673828125;
    public static final double DEPLOY_POSITION = 0.190673828125;
    public static final double CLIMB_POSITION = -0.12;

    public static final double CLIMB_POSITION_TOLERANCE = 0.005;

    public static final double RATCHET_SOLENOID_DEPLOY_PERCENTAGE = 1;

    private static final double CLIMBER_MAX_VELOCITY = 0.125;
    private static final double CLIMBER_MAX_ACCELERATION = CLIMBER_MAX_VELOCITY * 4;
    private static final double CLIMBER_MAX_JERK = CLIMBER_MAX_ACCELERATION * 4;

    public static final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs() // Unladen
                               .withGravityType(GravityTypeValue.Arm_Cosine)
                               .withKP(29.236)
                               .withKI(1)
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
                             .withNeutralMode(NeutralModeValue.Brake) // Would be better if we could get it to work with coast
                             .withInverted(InvertedValue.Clockwise_Positive))
        .withMotionMagic(new MotionMagicConfigs()
                             .withMotionMagicCruiseVelocity(CLIMBER_MAX_VELOCITY)
                             .withMotionMagicAcceleration(CLIMBER_MAX_ACCELERATION)
                             .withMotionMagicJerk(CLIMBER_MAX_JERK))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / MOTOR_TO_CLIMBER_GEAR_RATIO))
        .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}