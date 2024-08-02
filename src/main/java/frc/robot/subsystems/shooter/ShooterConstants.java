// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public final class ShooterConstants {
    
    public static final int kPivotMotorID = 30;
    public static final int kEncoderID = 2;

    public static final double kEncoderOffset = 0.25;

    public static final double kHandoffPosition = 0.0;
    public static final double kAmpPosition = 1.0;

    public static final int kLeftFlywheelMotorID = 31;
    public static final int kRightFlywheelMotorID = 32;
    public static final int kLeftIntakeMotorID = 23;
    public static final int kRightIntakeMotorID = 24;

    public static final double kRollersInSpeed = 1;
    public static final double kRollersOutSpeed = -1;
    public static final double kRollersShootSpeed = 1;

    public static final double kFlywheelShotSpeed = 1;

    public static final int kTopSensorID = 0;
    public static final int kSideSensorID = 2;
    public static final int kTopSensorThreshold = 1800;
    public static final int kSideSensorThreshold = 1200;

    public static final double kBangBangTolerance = 0.05;
    public static final double kPivotTolerance = 0.1;

    // Pivot gains
    public static final double dt = 0.02; // PID update period
    public static final double Ks = 0.0;
    public static final double Kv = 0.0;
    public static final double Kg = 0.0;
    public static final double Ka = 0.0;

    public static final double Kp = 0.0;
    public static final double Ki = 0.0;
    public static final double Kd = 0.0;

    public static final double[][] shotMatrix = {
        {0,0}
    };
    public static final double[][] passMatrix = {
        {0,0}
    };




}
