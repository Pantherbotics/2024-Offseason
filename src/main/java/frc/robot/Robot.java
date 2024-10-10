// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Handoff;
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

  private final CommandXboxController mainController = new CommandXboxController(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final Climber climber = new Climber();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Telemetry logger = new Telemetry();
  

  public Robot(){
    // default commands put subsystems in default state
    intake.setDefaultCommand(
      Commands.repeatingSequence(intake.pivotCtrCmd(IntakeConstants.kUpPosition), intake.rollerCtrlCmd(0))
    );
    shooter.setDefaultCommand(
      Commands.repeatingSequence(shooter.pivotCtrlCmd(ShooterConstants.kHandoffPosition), (shooter.rollerCtrlCmd(0)), (shooter.coastFlywheelsCmd()))
    );
    drivetrain.setDefaultCommand(drivetrain.applyRequest( 
      ()->DriveConstants.drive.withVelocityX(mainController.getLeftY() * DriveConstants.kMaxSpeed)
        .withVelocityY(mainController.getLeftX() * DriveConstants.kMaxSpeed) 
        .withRotationalRate(-mainController.getRightX() * DriveConstants.kMaxAngularRate)
    ));
    drivetrain.registerTelemetry(logger::telemeterize);
    drivetrain.invertEncoders();

    // sensor bindings
    intake.gotNote().onTrue(
      new Handoff(intake, shooter).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    );

    // Controller bindings
    mainController.leftBumper().toggleOnTrue(
      Commands.repeatingSequence(
        intake.pivotCtrCmd(IntakeConstants.kDownPosition),
        intake.rollerCtrlCmd(IntakeConstants.kInSpeed)
      )
    );
    
    mainController.rightBumper().onTrue(
      Commands.sequence(
        shooter.FlywheelCtrCmd(ShooterConstants.kFlywheelShotSpeed),
        shooter.pivotCtrlCmd(ShooterConstants.kSpeakerPosition),
        Commands.waitUntil(mainController.rightBumper().negate()),
        Commands.waitUntil(mainController.rightBumper()),
        shooter.rollerCtrlCmd(ShooterConstants.kRollersShootSpeed),
        Commands.waitUntil(()->!shooter.topSensor()).withTimeout(0.5),
        Commands.waitSeconds(0.5)
      )
    );

    mainController.button(7).onTrue(
      Commands.sequence(
        shooter.pivotCtrlCmd(ShooterConstants.kAmpPosition),
        shooter.FlywheelCtrCmd(0),
        shooter.rollerCtrlCmd(-0.5),
        Commands.waitSeconds(0.25),
        shooter.rollerCtrlCmd(0),
        Commands.waitUntil(mainController.button(7).negate()),
        Commands.waitUntil(mainController.button(7)),
        shooter.rollerCtrlCmd(ShooterConstants.kRollersOutSpeed),
        Commands.waitUntil(()->!shooter.topSensor()).withTimeout(0.5),
        Commands.waitSeconds(1)
      )
    );

    mainController.a().onTrue(
      climber.climbUntilSwitches().withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    );

    mainController.button(8).onTrue(
      intake.zeroIntake().alongWith(Commands.runOnce(null, shooter))
    );

    mainController.povDown().onTrue(
      Commands.runOnce(()->drivetrain.seedFieldRelative(), drivetrain)
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
