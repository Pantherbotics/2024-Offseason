// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterPivot;

public class RobotContainer {

  private final CommandJoystick mainController;
  private final CommandSwerveDrivetrain drivetrain;
  private final ShooterPivot shooterPivot;
  private final Intake intake;
  private final Climber climber;

  public RobotContainer() {
    drivetrain = TunerConstants.DriveTrain;
    mainController = new CommandJoystick(0);
    shooterPivot = new ShooterPivot();
    intake = new Intake();
    climber = new Climber();

    drivetrain.setDefaultCommand(DriveConstants.driveCommand(drivetrain, mainController.getHID()).ignoringDisable(true));
    configureBindings();
  }

  private void configureBindings() {
    mainController.button(1).whileTrue(shooterPivot.sysIdDynamicCommand(Direction.kForward));
    mainController.button(2).whileTrue(shooterPivot.sysIdDynamicCommand(Direction.kReverse));
    mainController.button(3).whileTrue(shooterPivot.sysIdQuasistaticCommand(Direction.kForward));
    mainController.button(4).whileTrue(shooterPivot.sysIdQuasistaticCommand(Direction.kReverse));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
