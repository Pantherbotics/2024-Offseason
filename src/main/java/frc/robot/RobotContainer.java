// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterPivot;

public class RobotContainer {

  private final CommandXboxController mainController;
  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterPivot shooterPivot;
  private final Intake intake;
  private final Climber climber;

  public RobotContainer() {
    drivetrain = TunerConstants.DriveTrain;
    mainController = new CommandXboxController(0);
    shooterPivot = new ShooterPivot();
    intake = new Intake();
    climber = new Climber();
    configureBindings();
  }

  private void configureBindings() {
    mainController.a().whileTrue(shooterPivot.sysIdDynamicCommand(Direction.kForward));
    mainController.x().whileTrue(shooterPivot.sysIdDynamicCommand(Direction.kReverse));
    mainController.b().whileTrue(shooterPivot.sysIdQuasistaticCommand(Direction.kForward));
    mainController.y().whileTrue(shooterPivot.sysIdQuasistaticCommand(Direction.kReverse));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
