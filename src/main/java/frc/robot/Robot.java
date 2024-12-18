// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Handoff;
import frc.robot.commands.noteAlignedDrive;
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
    drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(180));

    // sensor bindings
    mainController.x().onTrue(
      new Handoff(intake, shooter).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
      .andThen(Commands.runOnce(()->DataLogManager.log("manual handoff")))
    );

    intake.gotNote().onTrue(
      Commands.runOnce(()->mainController.getHID().setRumble(RumbleType.kBothRumble, 1)
        ).andThen(Commands.waitSeconds(0.5))
        .andThen(Commands.runOnce(()->mainController.getHID().setRumble(RumbleType.kBothRumble, 0))).alongWith(
        new Handoff(intake, shooter).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
      )
    );

    // Controller bindings
    mainController.leftBumper().toggleOnTrue(
      Commands.sequence(
        intake.pivotCtrCmd(IntakeConstants.kDownPosition),
        intake.rollerCtrlCmd(IntakeConstants.kInSpeed)
      ).alongWith(
        new noteAlignedDrive(drivetrain, mainController).until(mainController.y()).andThen(Commands.runOnce(()->DataLogManager.log("cancelled note align"))).andThen(
          drivetrain.applyRequest( 
      ()->DriveConstants.drive.withVelocityX(mainController.getLeftY() * DriveConstants.kMaxSpeed)
        .withVelocityY(mainController.getLeftX() * DriveConstants.kMaxSpeed) 
        .withRotationalRate(-mainController.getRightX() * DriveConstants.kMaxAngularRate)
        )
    
        )
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
        Commands.waitSeconds(1.5)
      )
    );

    mainController.a().onTrue(
      //shooter.pivotCtrlCmd(2.0).repeatedly().asProxy().alongWith(
      climber.ctrClimbersCmd(1)
      //)
    );

    new Trigger(climber::leftSwitch).onTrue(climber.ctrlLeftCmd(0));
    new Trigger(climber::rightSwitch).onTrue(climber.ctrlRightCmd(0).asProxy());

    mainController.button(8).onTrue(
      intake.zeroIntake().alongWith(Commands.runOnce(()->shooter.setFlywheelSpeed(0), shooter)).alongWith(Commands.runOnce(()->DataLogManager.log("zeroed intake")))
    );

    mainController.povDown().onTrue(
      Commands.runOnce(()->drivetrain.seedFieldRelative(new Pose2d()), drivetrain)
    );
  }

  @Override
  public void autonomousInit(){
    CommandScheduler.getInstance().schedule(
      Commands.sequence(
        shooter.FlywheelCtrCmd(ShooterConstants.kFlywheelShotSpeed),
        shooter.pivotCtrlCmd(ShooterConstants.kSpeakerPosition),
        Commands.waitSeconds(3),
        shooter.rollerCtrlCmd(ShooterConstants.kRollersShootSpeed),
        Commands.waitSeconds(1),
        shooter.rollerCtrlCmd(0),
        shooter.coastFlywheelsCmd()
        /*
        drivetrain.applyRequest(
          ()->
          new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(-1, 0, 0))
        ).withTimeout(3)
        */
      )
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
