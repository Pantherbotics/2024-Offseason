// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;


public class Robot extends TimedRobot {

  private final CommandXboxController mainController = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final Climber climber = new Climber();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Telemetry logger = new Telemetry();

  public Robot(){

    intake.setDefaultCommand(
      intake.pivotCtrCmd(IntakeConstants.kUpPosition).andThen(intake.rollerCtrlCmd(0)).repeatedly()
    );
    shooter.setDefaultCommand(
      shooter.pivotCtrlCmd(ShooterConstants.kHandoffPosition).andThen(shooter.rollerCtrlCmd(0)).andThen(shooter.coastFlywheelsCmd()).repeatedly()
    );
    drivetrain.setDefaultCommand(drivetrain.applyRequest( 
      ()->DriveConstants.drive.withVelocityX(-mainController.getLeftX() * DriveConstants.kMaxSpeed)
        .withVelocityY(-mainController.getLeftY() * DriveConstants.kMaxSpeed) 
        .withRotationalRate(-mainController.getRightX() * DriveConstants.kMaxAngularRate)
    ));

    mainController.leftBumper().onTrue(
      intake.pivotCtrCmd(IntakeConstants.kDownPosition).andThen(intake.rollerCtrlCmd(IntakeConstants.kInSpeed))
    );

    


  }

  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }



}
