// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

/** Add your docs here. */
public final class ShooterConstants {
    
    public static final int kPivotMotorID = 30;
    public static final int kEncoderID = 2;

    public static final double kEncoderOffset = 0.0;

    public static final double kHandoffPosition = 0.0;
    public static final double kAmpPosition = 1.0;

    public static final int kLeftFlywheelMotorID = 20;
    public static final int kRightFlywheelMotorID = 21;
    public static final int kLeftIntakeMotorID = 23;
    public static final int kRightIntakeMotorID = 24;

    public static final int kTopSensorID = 3;
    public static final int kSideSensorID = 4;
    public static final int kTopSensorThreshold = 0;
    public static final int kSideSensorThreshold = 0;

    public static final double kBangBangTolerance = 0.05;

    // Pivot gains
    public static final double dt = 0.02; // PID update period
    public static final double Ks = 0.0;
    public static final double Kv = 0.0;
    public static final double Kg = 0.0;
    public static final double Ka = 0.0;

    public static final double Kp = 0.0;
    public static final double Ki = 0.0;
    public static final double Kd = 0.0;
}
