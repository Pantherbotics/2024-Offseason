// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

/** Add your docs here. */
public class IntakeConstants {
    public static final int kPivotMotorID = 25;
    public static final int kRollersMotorID = 24;

    public static final double kGoalTolerance = 0.025;

    public static final int kLimitSiwtchID = 0;

    public static final int kBeamBreakID = 3;
    public static final double kDebounceTime = 0.12;

    public static final double kDownPosition = 1.35;
    public static final double kUpPosition = 0;
    public static final double kInSpeed = -1;
    public static final double kOutSpeed = 0.3;

    
    public static final double kMotorToPivotRatio = 64.0 * (23.0/42.0);

    public static Slot0Configs kPivotGains = new Slot0Configs()
    .withKS(0.10809)
    .withKV(3.7321)
    .withKG(0.032992)
    .withKA(0.044371)
    .withKP(65.454)
    .withKD(5.3774)
    .withGravityType(GravityTypeValue.Arm_Cosine);

    public static MotionMagicConfigs kProfileConfigs = new MotionMagicConfigs()
    .withMotionMagicCruiseVelocity(50)
    .withMotionMagicAcceleration(15)
    .withMotionMagicJerk(34);



}
