// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class DriveConstants {

    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAngularRate = 1.77 * Math.PI;

    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // main drive type
      .withDeadband(kMaxSpeed * 0.05).withRotationalDeadband(kMaxAngularRate * 0.05)
      .withDriveRequestType(DriveRequestType.Velocity);

    public Command driveCommand(CommandSwerveDrivetrain drivetrain, XboxController joystick){
      return drivetrain.applyRequest(
        () -> drive.withVelocityX(-joystick.getLeftY() * kMaxSpeed)
        .withVelocityY(-joystick.getLeftX() * kMaxSpeed) 
        .withRotationalRate(-joystick.getRightX() * kMaxAngularRate)
      );
    }


    public static final PathConstraints kPathfindingConstraints = new PathConstraints(
        2, 1.75,
        Units.degreesToRadians(180), Units.degreesToRadians(180));

}
