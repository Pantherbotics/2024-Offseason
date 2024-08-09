// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.DriverIO;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.FieldPoses;

public class Amp extends Command {
  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;
  private final DriverIO mainIO;
  private boolean ampButton = true;
  private boolean amping = false;
  private double time;

  public Amp(Shooter shooter, CommandSwerveDrivetrain drivetrain, DriverIO mainIO) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.mainIO = mainIO;
    addRequirements(shooter, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ampButton = true;
    amping = false;
    shooter.setPivotGoal(ShooterConstants.kAmpPosition);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ampButton && mainIO.amp().getAsBoolean() && shooter.isAtGoal() && !amping){
      amping = true;
      shooter.setRollers(ShooterConstants.kRollersOutSpeed);
      time = Utils.getCurrentTimeSeconds();
    }

    if (amping) {
      shooter.setRollers(ShooterConstants.kRollersOutSpeed);
      shooter.setPivotGoal(ShooterConstants.kAmpPosition + mainIO.wiggleAmp()/30);
    }

    
    drivetrain.setControl(
      DriveConstants.facing
      .withVelocityX(-mainIO.moveY() * DriveConstants.kMaxSpeed)
      .withVelocityY(-mainIO.moveX() * DriveConstants.kMaxSpeed)
      .withTargetDirection(Rotation2d.fromDegrees(FieldPoses.isRedAlliance?-90:90))
    );
    
    
    ampButton = mainIO.amp().getAsBoolean();
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRollers(0);
    shooter.setPivotGoal(ShooterConstants.kHandoffPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return amping && Utils.getCurrentTimeSeconds() - time > 1;
  }
}
