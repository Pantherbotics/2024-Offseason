// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.intake.Intake;

public class IntakeAssist extends Command {
  private final Intake intake;
  private final CommandSwerveDrivetrain drivetrain;

  public IntakeAssist(Intake m_intake, CommandSwerveDrivetrain m_drivetrain) {
    this.intake = m_intake;
    this.drivetrain = m_drivetrain;
    addRequirements(m_intake, m_drivetrain);
  }

  @Override
  public void initialize() {
    intake.setPivotDown();
    intake.setRollersIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(DriveConstants.drive
      .withVelocityX(0.0)
      
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
