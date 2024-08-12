// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.DriverIO;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.FieldPoses;

public class simpleShot extends Command {
  private final Shooter shooter;
  private final DriverIO mainIO;
  private boolean shootButton = true;
  private boolean justShot = false;
  private double pivotAngle = 0;
  private Debouncer m_debouncer = new Debouncer(0.5, DebounceType.kFalling);

  public simpleShot(Shooter shooter, DriverIO mainIO) {
    this.shooter = shooter;
    this.mainIO = mainIO;
    addRequirements(shooter);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setFlywheelSpeed(ShooterConstants.kFlywheelShotSpeed);
    shooter.setRollers(0);
    shootButton = true;
    justShot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotAngle += mainIO.wiggleAmp()/50;
    shooter.setPivotGoal(pivotAngle);
    if (!shootButton && mainIO.shoot().getAsBoolean()){
      justShot = true;
    }

    if (justShot) {
      shooter.setRollers(ShooterConstants.kRollersShootSpeed);
    }

    
    shootButton = mainIO.shoot().getAsBoolean();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("shot distance", TunerConstants.DriveTrain.getState().Pose.relativeTo(FieldPoses.kSpeakerPose).getTranslation().getNorm());
    SmartDashboard.putNumber("shot angle", shooter.pivotAngle());

    shooter.setRollers(0);
    shooter.setFlywheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_debouncer.calculate(shooter.noteSeated()) && !Utils.isSimulation();
  }
}
