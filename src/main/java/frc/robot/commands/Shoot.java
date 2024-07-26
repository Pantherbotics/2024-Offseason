// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.proto.Geometry2D;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.DriverIO;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.FieldPoses;

public class Shoot extends Command {
  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;
  private final DriverIO mainIO;

  public Shoot(Shooter shooter, CommandSwerveDrivetrain drivetrain, DriverIO mainIO) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.mainIO = mainIO;
    addRequirements(shooter, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setFlywheelSpeed(ShooterConstants.kFlywheelShotSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = drivetrain.getState().Pose;
    Pose2d speakerPose = FieldPoses.kSpeakerPose;
    
    Rotation2d angleToSpeaker = new Rotation2d(speakerPose.getX() - robotPose.getX(),speakerPose.getY() - robotPose.getY()).plus(Rotation2d.fromDegrees(180));//Rotation2d.fromRadians(Math.atan2(speakerPose.getY() - robotPose.getY(), speakerPose.getX() - robotPose.getX()));
    drivetrain.setControl(
      DriveConstants.facing.withTargetDirection(angleToSpeaker)
      .withVelocityX(-mainIO.moveY() * DriveConstants.kMaxSpeed)
      .withVelocityY(-mainIO.moveX() * DriveConstants.kMaxSpeed)
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
