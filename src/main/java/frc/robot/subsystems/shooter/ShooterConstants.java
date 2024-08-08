// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

/** Add your docs here. */
public final class ShooterConstants {
    
    public static final int kPivotMotorID = 30;
    public static final int kEncoderID = 33;

    public static final double kEncoderOffset = -0.16;

    public static final double kHandoffPosition = -0.06;
    public static final double kAmpPosition = 0.13;

    public static final int kLeftFlywheelMotorID = 31;
    public static final int kRightFlywheelMotorID = 32;
    public static final int kLeftIntakeMotorID = 22;
    public static final int kRightIntakeMotorID = 23;

    public static final double kRollersInSpeed = -0.3;
    public static final double kRollersOutSpeed = 1;
    public static final double kRollersShootSpeed = -1;

    public static final double kFlywheelShotSpeed = 100;

    public static final int kTopSensorID = 0;
    public static final int kSideSensorID = 2;
    public static final int kTopSensorThreshold = 1800;
    public static final int kSideSensorThreshold = 1200;

    public static final double kBangBangTolerance = 0.05;
    public static final double kPivotTolerance = 0.1;
    public static final double kMotorToPivotRatio =117.87;// 64.0 * (23.0/42.0);

    public static Slot0Configs kPivotGains = new Slot0Configs()
    .withKS(0.15)
    .withKV(9)
    .withKG(0.07)
    .withKA(0.05)
    .withKP(35)
    .withKD(3)
    .withGravityType(GravityTypeValue.Arm_Cosine);

    public static MotionMagicConfigs kProfileConfigs = new MotionMagicConfigs()
    .withMotionMagicCruiseVelocity(200)
    .withMotionMagicAcceleration(35)
    .withMotionMagicJerk(200);

    
    public static FeedbackConfigs kFeedbackConfigs = new FeedbackConfigs()
    .withFeedbackRemoteSensorID(kEncoderID)
    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
    .withRotorToSensorRatio(kMotorToPivotRatio);


    public static final double[][] shotMatrix = {
        {1.38,-0.093},
        {1.63,-0.1},
        {2,-0.13},
        {2.67,-0.16},
        {3.33,-0.17}
    };
    public static final double[][] passMatrix = {
        {0,0}
    };




}
