// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.DriverIO;

/** Add your docs here. */
public class DriveConstants {

    public static final double kMaxSpeed = 4.0;
    public static final double kMaxAngularRate = 1.5 * Math.PI;

    public static final Pose2d kAmpPose = new Pose2d(1.8,7.65, Rotation2d.fromDegrees(90));
    public static final Pose2d kSpeakerPose = new Pose2d(0,5.5, Rotation2d.fromDegrees(90));


    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // main drive type
      .withDeadband(kMaxSpeed * 0.05).withRotationalDeadband(kMaxAngularRate * 0.05)
      .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);


    public static final SwerveRequest.FieldCentricFacingAngle facing = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(kMaxSpeed * 0.05).withRotationalDeadband(kMaxAngularRate * 0.05)
      .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);

      
    public static Command driveCommand(CommandSwerveDrivetrain drivetrain, DriverIO mainIO){
      return drivetrain.applyRequest(
        () -> drive.withVelocityX(-mainIO.moveY() * kMaxSpeed)
        .withVelocityY(-mainIO.moveX() * kMaxSpeed) 
        .withRotationalRate(-mainIO.rotate() * kMaxAngularRate)
      );
    }

    public static final PhoenixPIDController kHeadingController = new PhoenixPIDController(1, 0.0, 0.1);
    

    public static final PathConstraints kPathfindingConstraints = new PathConstraints(
        6, 4,
        Units.degreesToRadians(360), Units.degreesToRadians(180));

    public static final PIDConstants kMoveGains = new PIDConstants(10, 0, 0);
    public static final PIDConstants kTurnGains = new PIDConstants(10, 0, 0);

}
