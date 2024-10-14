// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;

public class noteAlignedDrive extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  /** Creates a new noteAlignedDrive. */
  public noteAlignedDrive(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.setControl(
      DriveConstants.drive
      .withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(0)
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
