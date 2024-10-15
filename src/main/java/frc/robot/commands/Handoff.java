// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Handoff extends SequentialCommandGroup {

  public Handoff(Intake intake, Shooter shooter) {
    
    addCommands(
      intake.rollerCtrlCmd(0),
      intake.pivotCtrCmd(IntakeConstants.kUpPosition),
      shooter.pivotCtrlCmd(ShooterConstants.kHandoffPosition),
      new WaitUntilCommand(intake::isAtGoal),
      new WaitCommand(0.1),
      intake.rollerCtrlCmd(IntakeConstants.kOutSpeed),
      shooter.rollerCtrlCmd(ShooterConstants.kRollersInSpeed),
      new WaitUntilCommand(shooter::topSensor).withTimeout(1).finallyDo((end)->{if(end){DataLogManager.log("shooter note loading failed");CommandScheduler.getInstance().cancel(this);}}),
      shooter.rollerCtrlCmd(0),
      intake.rollerCtrlCmd(0)
    );
    
  }
}
