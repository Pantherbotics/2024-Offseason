// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {

    public static final double kMaxSpeed = 4;
    public static final double kMaxAngularRate = 1.5 * Math.PI;

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // main drive type
      .withDeadband(kMaxSpeed * 0.01).withRotationalDeadband(kMaxAngularRate * 0.01)
      .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagic);

    public static final PathConstraints kPathfindingConstraints = new PathConstraints(
        6, 4,
        Units.degreesToRadians(360), Units.degreesToRadians(180));

    public static final PIDConstants kMoveGains = new PIDConstants(10, 0, 0);
    public static final PIDConstants kTurnGains = new PIDConstants(10, 0, 0);

}
