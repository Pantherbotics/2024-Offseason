// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.vision.LimelightHelpers;

public class noteAlignedDrive extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController mainController;
  private PIDController controller = new PIDController(0.75, 0, 0);
  private final double kP = 0.05;

  /** Creates a new noteAlignedDrive. */
  public noteAlignedDrive(CommandSwerveDrivetrain drivetrain, CommandXboxController xboxController) {
    this.drivetrain = drivetrain;
    this.mainController = xboxController;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {

    var pose = drivetrain.getState().Pose;

    double calculated = controller.calculate(-LimelightHelpers.getTX("") * kP);

    var notespeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0.0,calculated,0.0, pose.getRotation());

    double driveSpeed = Math.hypot(drivetrain.getState().speeds.vxMetersPerSecond, drivetrain.getState().speeds.vyMetersPerSecond);

    drivetrain.setControl(
      DriveConstants.drive
      .withVelocityX(mainController.getLeftY() * DriveConstants.kMaxSpeed + notespeeds.vxMetersPerSecond * driveSpeed/DriveConstants.kMaxSpeed)
      .withVelocityY(mainController.getLeftX() * DriveConstants.kMaxSpeed + notespeeds.vyMetersPerSecond * driveSpeed/DriveConstants.kMaxSpeed)
      .withRotationalRate(-mainController.getRightX() * DriveConstants.kMaxAngularRate)
    );

    SmartDashboard.putNumber("Note Align",calculated);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
