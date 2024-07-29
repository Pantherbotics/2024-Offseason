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

    public static final double kGoalTolerance = 0.1;

    public static final int kLimitSiwtchID = 3;

    public static final int kDistanceSensorID = 3;
    public static final int kSensorThreshold = 600;

    public static final double kDownPosition = 0.25;
    public static final double kUpPosition = 0.0;
    public static final double kInSpeed = 1.0;
    public static final double kOutSpeed = -1.0;

    
    public static final double kMotorToPivotRatio = 0.0;
    public static final double kRotorOffset = 0.0;

    public static Slot0Configs kPivotGains = new Slot0Configs()
    .withKS(0.0)
    .withKV(0.0)
    .withKG(0.0)
    .withKA(0.0)
    .withKP(0.0)
    .withKD(0.0)
    .withGravityType(GravityTypeValue.Arm_Cosine);

    public static MotionMagicConfigs kProfileConfigs = new MotionMagicConfigs()
    .withMotionMagicCruiseVelocity(80)
    .withMotionMagicAcceleration(160)
    .withMotionMagicJerk(1600);



}
