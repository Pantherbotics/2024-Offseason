// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class DriveConstants {

    public static final double kMaxSpeed = 4.0;
    public static final double kMaxAngularRate = 1.5 * Math.PI;

    public static final Pose2d kAmpPose = new Pose2d(1.8,7.65, Rotation2d.fromDegrees(90));

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // main drive type
      .withDeadband(kMaxSpeed * 0.05).withRotationalDeadband(kMaxAngularRate * 0.05)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public static Command driveCommand(CommandSwerveDrivetrain drivetrain, Joystick joystick){
      return drivetrain.applyRequest(
        () -> drive.withVelocityX(-joystick.getY() * kMaxSpeed)
        .withVelocityY(-joystick.getX() * kMaxSpeed) 
        .withRotationalRate(-joystick.getZ() * kMaxAngularRate)
      );
    }


    public static final PathConstraints kPathfindingConstraints = new PathConstraints(
        6, 4,
        Units.degreesToRadians(360), Units.degreesToRadians(180));

    public static final PIDConstants kMoveGains = new PIDConstants(10, 0, 0);
    public static final PIDConstants kTurnGains = new PIDConstants(10, 0, 0);

}
